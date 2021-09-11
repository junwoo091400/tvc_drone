#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/ActuatorControl.h>
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/DroneState.h"

#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#include <boost/array.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>

#include <nav_msgs/Odometry.h>


#include <mavros_msgs/RCIn.h>
#include <algorithm>

//using namespace Eigen;

geometry_msgs::Vector3 current_target_apogee;
void targetCallback(const geometry_msgs::Vector3 &target) {
    current_target_apogee = target;
}


mavros_msgs::State current_state;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


ros::Publisher pixhawk_control_pub, pixhawk_state_pub, target_apogee_pub;
boost::array<double, 8> pixhawk_controls;
double max_servo_angle = 15.0/180.0*M_PI;

double servo1_offset;
double servo2_offset;

void updateCurrentControl(const drone_gnc::DroneControl::ConstPtr &drone_control) {
    double servo1 = std::min(std::max(drone_control->servo1, -max_servo_angle), max_servo_angle);
    double servo2 = std::min(std::max(drone_control->servo2, -max_servo_angle), max_servo_angle);

    double bottom = std::min(std::max(drone_control->bottom, 0.0), 90.0);
    double top = std::min(std::max(drone_control->top, 0.0), 90.0);

    pixhawk_controls[0] = bottom/100*2 - 1;
    pixhawk_controls[1] = top/100*2 - 1;
    pixhawk_controls[2] = servo1/(M_PI/4) + servo1_offset;
    pixhawk_controls[3] = servo2/(M_PI/4) + servo2_offset;
}

void publishConvertedState(const nav_msgs::Odometry::ConstPtr& mavros_state) {
    drone_gnc::DroneState drone_state;
    drone_state.header.stamp = ros::Time::now();
    drone_state.pose = mavros_state->pose.pose;
    drone_state.twist = mavros_state->twist.twist;

    pixhawk_state_pub.publish(drone_state);
}

double joystic_speed = 1;
double last_rc_cb;
double max_z = 2;
void RCCallback(const mavros_msgs::RCIn::ConstPtr& rc_control) {
    double dt = ros::Time::now().toSec() - last_rc_cb;
    double joystic_x = ((double)rc_control->channels[1]-1500)/1000;
    double joystic_y = ((double)rc_control->channels[2]-1500)/1000;
    double joystic_z = ((double)rc_control->channels[0]-1000)/1000;

    if(abs(joystic_x)<0.1)joystic_x=0;
    if(abs(joystic_y)<0.1)joystic_y=0;

    geometry_msgs::Vector3 new_target_apogee;
    new_target_apogee.x = current_target_apogee.x + joystic_x * joystic_speed*dt;
    new_target_apogee.y = current_target_apogee.y + joystic_y * joystic_speed*dt;
    new_target_apogee.z = joystic_z*max_z;
    last_rc_cb = ros::Time::now().toSec();
    target_apogee_pub.publish(new_target_apogee);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_mavros_interface");
    ros::NodeHandle nh("drone_mavros_interface");

    double servo1_offset_degree, servo2_offset_degree;
    nh.param<double>("servo1_offset", servo1_offset_degree, 0.0);
    nh.param<double>("servo2_offset", servo2_offset_degree, 0.0);
    servo1_offset = servo1_offset_degree*M_PI/180.0;
    servo2_offset = servo2_offset_degree*M_PI/180.0;

    // Subscribe to both mavros pose and twist and merge them
    // message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/mavros/local_position/pose", 1);
    // message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(nh, "/mavros/local_position/velocity_body", 1);
    // message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sync(pose_sub, vel_sub,10);
    // sync.registerCallback(boost::bind(&publishConvertedState, _1, _2));
    ros::Subscriber pose_sub = nh.subscribe("/mavros/global_position/local", 1, publishConvertedState);

    // Subscribe to drone control
    ros::Subscriber drone_control_sub = nh.subscribe("/drone_control", 1, updateCurrentControl);

    // Subscribe to drone control
    ros::Subscriber target_apogee_sub = nh.subscribe("/target_apogee", 1, targetCallback);

    // Create control publisher
    pixhawk_control_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

    target_apogee_pub = nh.advertise<geometry_msgs::Vector3>("/target_apogee", 10);


    // Subscribe to drone control
    last_rc_cb = ros::Time::now().toSec();
    bool enable_rc_control;
    nh.param<bool>("enable_rc_control", enable_rc_control, false);
    ros::Subscriber rc_sub;
    if (enable_rc_control){
         rc_sub = nh.subscribe("/mavros/rc/in", 1, RCCallback);
    }

    // Create state publisher
    pixhawk_state_pub = nh.advertise<drone_gnc::DroneState>("/pixhawk_drone_state", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, mavrosStateCallback);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    ros::Rate rate(200.0);
    for(int i = 100; ros::ok() && i > 0; --i){
        pixhawk_controls[0] = -1;
        pixhawk_controls[1] = -1;
        pixhawk_controls[2] = -0.02;
        pixhawk_controls[3] = 0.24;

        mavros_msgs::ActuatorControl pixhawk_control_msg;
        pixhawk_control_msg.header.stamp = ros::Time::now();
        pixhawk_control_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;
        pixhawk_control_msg.controls = pixhawk_controls;

        pixhawk_control_pub.publish(pixhawk_control_msg);
        ros::spinOnce();
        rate.sleep();
    }


    // Automatic callback of service and publisher from here
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        mavros_msgs::ActuatorControl pixhawk_control_msg;
        pixhawk_control_msg.header.stamp = ros::Time::now();
        pixhawk_control_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;
        pixhawk_control_msg.controls = pixhawk_controls;

        pixhawk_control_pub.publish(pixhawk_control_msg);

        ros::spinOnce();
        rate.sleep();
    }
}
