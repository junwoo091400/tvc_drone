#include "ros/ros.h"

#include "drone_gnc/DroneControl.h"
#include "real_time_simulator/State.h"
#include "drone_gnc/DroneState.h"
#include "real_time_simulator/Control.h"
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <random>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include "drone_model.hpp"

using namespace Eigen;

ros::Publisher rocket_control_pub, drone_state_pub, pixhawk_state_pub, command_pub, fake_optitrack_pub,
        fake_pixhawk_pose_pub, fake_pixhawk_twist_local_pub, fake_pixhawk_twist_body_pub, fake_gps_pub;

double CM_to_thrust_distance;


Matrix<double, 2, 2> sysA;
Matrix<double, 2, 1> sysB;
Matrix<double, 1, 2> sysC;
Matrix<double, 2, 1> x_servo1;
Matrix<double, 2, 1> x_servo2;

double CM_OFFSET_X = 0;

bool first_command = true;
double thrust_scaling, torque_scaling, servo1_offset, servo2_offset;
Drone *drone;

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);

std::normal_distribution<double> servo_noise(0.0, 0.0);
std::normal_distribution<double> torque_noise(0.0, 0.0);

double last_control_time;
bool use_servo_model;

void publishConvertedControl(const drone_gnc::DroneControl::ConstPtr &drone_control) {
    double time_now = ros::Time::now().toSec();
    double delta_t = time_now - last_control_time;
    delta_t = 0.02;
    last_control_time = time_now;

    double thrust = thrust_scaling * drone->getThrust((drone_control->bottom + drone_control->top) * 0.5);
    double torque = torque_scaling * drone->getTorque(drone_control->top - drone_control->bottom);

    double servo1_cmd = drone_control->servo1;
    double servo2_cmd = drone_control->servo2;

    // ss model, euler discretization
    x_servo1 += (sysA * x_servo1 + sysB * servo1_cmd) * delta_t;
    x_servo2 += (sysA * x_servo2 + sysB * servo2_cmd) * delta_t;

    double servo1, servo2;
    if (use_servo_model) {
        servo1 = sysC * x_servo1;
        servo2 = sysC * x_servo2;
    } else {
        servo1 = servo1_cmd;
        servo2 = servo2_cmd;
    }

    //quaternion representing the rotation of the servos around the Y-axis followed by the rotation around the X-axis
    Eigen::Quaterniond
            thrust_rotation(
            AngleAxisd(servo1 + servo1_offset + servo_noise(generator), Vector3d::UnitX()) *
            AngleAxisd(servo2 + servo2_offset + servo_noise(generator), Vector3d::UnitY())
    );

    //rotated thrust vector, in body frame
    Eigen::Vector3d thrust_direction = thrust_rotation._transformVector(Vector3d::UnitZ());
    Eigen::Vector3d thrust_vector = thrust_direction * thrust;

    Eigen::Vector3d propeller_torque = thrust_direction * torque;

    //compute the force and torque at the center of mass and publish them
    real_time_simulator::Control converted_control;


    converted_control.torque.x =
            thrust_vector.y() * CM_to_thrust_distance + propeller_torque.x() + torque_noise(generator);
    converted_control.torque.y =
            -thrust_vector.x() * CM_to_thrust_distance + propeller_torque.y() - CM_OFFSET_X * thrust_vector.z() +
            torque_noise(generator);
    converted_control.torque.z = torque + propeller_torque.z() + CM_OFFSET_X * thrust_vector.y();

    converted_control.force.x = thrust_vector.x();
    converted_control.force.y = thrust_vector.y();
    converted_control.force.z = thrust_vector.z();
    rocket_control_pub.publish(converted_control);

    if (first_command) {
        first_command = false;
        std_msgs::String msg;
        msg.data = "Launch";
        command_pub.publish(msg);
    }
}

//double t_start = 0;
void publishConvertedState(const real_time_simulator::State::ConstPtr &rocket_state) {

    //simulator uses angular vel in inertial frame while mpc uses body frame
    Eigen::Quaterniond attitude(rocket_state->pose.orientation.w, rocket_state->pose.orientation.x,
                                rocket_state->pose.orientation.y, rocket_state->pose.orientation.z);
    Eigen::Vector3d omega_inertial(rocket_state->twist.angular.x, rocket_state->twist.angular.y,
                                   rocket_state->twist.angular.z);
    Eigen::Vector3d omega_body = attitude.inverse()._transformVector(omega_inertial);
    geometry_msgs::Vector3 omega_body_msg;
    omega_body_msg.x = omega_body(0);
    omega_body_msg.y = omega_body(1);
    omega_body_msg.z = omega_body(2);

//    converted_state.twist.angular.x = omega_inertial(0);
//    converted_state.twist.angular.y = omega_inertial(1);
//    converted_state.twist.angular.z = omega_inertial(2);
//    if(t_start == 0){
//        t_start = ros::Time::now().toSec();
//    }
//    double t = ros::Time::now().toSec() - t_start;
//    if(t < 1){
//        drone_gnc::DroneState state_msg;
//        state_msg.pose.position.x = sin(10*t);
//        state_msg.twist.linear.x = 10*cos(10*t);
//        state_msg.pose.orientation.w = 1;
//        state_msg.header.stamp = ros::Time(t);
//        drone_state_pub.publish(state_msg);
//
//        geometry_msgs::PoseStamped pose_msg;
//        pose_msg.pose.position.x = -sin(10*t);
//        pose_msg.pose.orientation.w = 1;
//        pose_msg.header.stamp = ros::Time(t);
//        fake_optitrack_pub.publish(pose_msg);
//        ROS_ERROR_STREAM("t: " << t);
//    }

    ros::Time now = ros::Time::now();
    drone_gnc::DroneState converted_state;
    converted_state.twist.angular = omega_body_msg;
    converted_state.twist.linear = rocket_state->twist.linear;
    converted_state.pose = rocket_state->pose;
    converted_state.thrust_scaling = 1;
    converted_state.torque_scaling = 1;
    converted_state.header.stamp = now;

    drone_state_pub.publish(converted_state);
    pixhawk_state_pub.publish(converted_state);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.orientation = converted_state.pose.orientation;
    pose_msg.pose.position.x = -converted_state.pose.position.x;
    pose_msg.pose.position.y = -converted_state.pose.position.y;
    pose_msg.pose.position.z = converted_state.pose.position.z;
    pose_msg.header.stamp = now;

    geometry_msgs::TwistStamped twist_msg_local;
    twist_msg_local.twist.linear = converted_state.twist.linear;
    twist_msg_local.header.stamp = now;

    geometry_msgs::TwistStamped twist_msg_body;
    twist_msg_body.twist.angular = converted_state.twist.angular;
    twist_msg_body.header.stamp = now;

    nav_msgs::Odometry gps_msg;
    gps_msg.pose.pose = converted_state.pose;
    gps_msg.twist.twist.angular = converted_state.twist.angular;
    gps_msg.twist.twist.linear.x = converted_state.twist.linear.x;
    gps_msg.twist.twist.linear.y = converted_state.twist.linear.y;
    gps_msg.twist.twist.linear.z = -converted_state.twist.linear.z;
    gps_msg.header.stamp = now;

    fake_optitrack_pub.publish(pose_msg);
    fake_pixhawk_pose_pub.publish(pose_msg);
    fake_pixhawk_twist_local_pub.publish(twist_msg_local);
    fake_pixhawk_twist_body_pub.publish(twist_msg_body);
    fake_gps_pub.publish(gps_msg);
}

int main(int argc, char **argv) {
    //init state system (discrete ss model for ts = 0.03s)
    x_servo1 << 0, 0;
    x_servo2 << 0, 0;
    sysA << -21.94, -15.34,
            16, 0;
    sysB << 4,
            0;
    sysC << 0, 3.823;

    // Init ROS time keeper node
    ros::init(argc, argv, "simulator_interface");
    ros::NodeHandle nh("simulator_interface");

    Drone drone_obj(nh);
    drone = &drone_obj;

    nh.param<double>("/rocket/estimated/thrust_scaling", thrust_scaling, 1);
    nh.param<double>("/rocket/estimated/torque_scaling", torque_scaling, 1);
    nh.param<double>("/rocket/estimated/servo1_offset", servo1_offset, 0);
    nh.param<double>("/rocket/estimated/servo2_offset", servo2_offset, 0);

    nh.param<bool>("use_servo_model", use_servo_model, false);

    //TODO check if exists
    nh.getParam("/rocket/CM_to_thrust_distance", CM_to_thrust_distance);
    nh.getParam("/rocket/CM_offset_x", CM_OFFSET_X);

    // Subscribe to drone control
    ros::Subscriber drone_control_sub = nh.subscribe("/drone_control", 10, publishConvertedControl);

    // Create control publisher
    rocket_control_pub = nh.advertise<real_time_simulator::Control>("/control_measured", 10);

    fake_optitrack_pub = nh.advertise<geometry_msgs::PoseStamped>("/optitrack_client/Drone/optitrack_pose", 10);
    fake_pixhawk_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10);
    fake_pixhawk_twist_local_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",
                                                                             10);
    fake_pixhawk_twist_body_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10);

    fake_gps_pub = nh.advertise<nav_msgs::Odometry>("/mavros/global_position/local", 10);


    // Subscribe to rocket state
    ros::Subscriber rocket_state_sub = nh.subscribe("/rocket_state", 10, publishConvertedState);

    // Create drone state publisher
    drone_state_pub = nh.advertise<drone_gnc::DroneState>("/simu_drone_state", 10);
//    pixhawk_state_pub = nh.advertise<drone_gnc::DroneState>("/pixhawk_drone_state", 10);

    command_pub = nh.advertise<std_msgs::String>("/commands", 10);
    // Automatic callback of service and publisher from here
    ros::spin();
}
