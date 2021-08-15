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

//using namespace Eigen;

ros::Publisher pixhawk_control_pub, pixhawk_state_pub;
boost::array<double, 8> pixhawk_controls;

void publishConvertedControl(const drone_gnc::DroneControl::ConstPtr &drone_control) {
    pixhawk_controls.at(0) = drone_control->bottom;
    pixhawk_controls.at(1) = drone_control->top;
    pixhawk_controls.at(2) = drone_control->servo1;
    pixhawk_controls.at(3) = drone_control->servo2;

    mavros_msgs::ActuatorControl pixhawk_control_msg;
    pixhawk_control_msg.header.stamp = ros::Time::now();
    pixhawk_control_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_PAYLOAD;
    pixhawk_control_msg.controls = pixhawk_controls;

    pixhawk_control_pub.publish(pixhawk_control_msg);
}

void publishConvertedState(const geometry_msgs::PoseStamped::ConstPtr &mavros_pose,
                           const geometry_msgs::TwistStamped::ConstPtr &mavros_twist) {
    drone_gnc::DroneState drone_state;
    drone_state.header.stamp = ros::Time::now();
    drone_state.pose = mavros_pose->pose;
    drone_state.twist = mavros_twist->twist;

    pixhawk_state_pub.publish(drone_state);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_mavros_interface");
    ros::NodeHandle nh("drone_mavros_interface");

    // Subscribe to both mavros pose and twist and merge them
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/mavros/local_position/pose", 1);
    message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(nh, "/mavros/local_position/velocity", 1);
    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sync(pose_sub, vel_sub,10);
    sync.registerCallback(boost::bind(&publishConvertedState, _1, _2));

    // Subscribe to drone control
    ros::Subscriber drone_control_sub = nh.subscribe("/drone_control", 10, publishConvertedControl);

    // Create control publisher
    pixhawk_control_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

    // Create state publisher
    pixhawk_state_pub = nh.advertise<drone_gnc::DroneState>("/pixhawk_drone_state", 10);

    // Automatic callback of service and publisher from here
    ros::spin();
}
