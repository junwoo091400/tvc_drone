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

geometry_msgs::Vector3 current_target_apogee;
void DroneControlNode::targetCallback(const geometry_msgs::Vector3 &target) {
//    const std::lock_guard<std::mutex> lock(target_mutex);
    current_target_apogee = target;
}


mavros_msgs::State current_state;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



//using namespace Eigen;

ros::Publisher pixhawk_control_pub, pixhawk_state_pub, target_apogee_pub;
boost::array<double, 8> pixhawk_controls;

void updateCurrentControl(const drone_gnc::DroneControl::ConstPtr &drone_control) {
    pixhawk_controls.at(0) = drone_control->bottom;
    pixhawk_controls.at(1) = drone_control->top;
    pixhawk_controls.at(2) = drone_control->servo1;
    pixhawk_controls.at(3) = drone_control->servo2;
}

void publishConvertedState(const geometry_msgs::PoseStamped::ConstPtr &mavros_pose,
                           const geometry_msgs::TwistStamped::ConstPtr &mavros_twist) {
    drone_gnc::DroneState drone_state;
    drone_state.header.stamp = ros::Time::now();
    drone_state.pose = mavros_pose->pose;
    drone_state.twist = mavros_twist->twist;

    pixhawk_state_pub.publish(drone_state);
}

void RCCallback(const mavros_msgs::ManualControl::ConstPtr& rc_control) {
    geometry_msgs::Vector3 new_target_apogee;
    new_target_apogee.x = rc_control->x;
    new_target_apogee.y = rc_control->y;
    new_target_apogee.z = rc_control->z;
    target_apogee_pub.publish(new_target_apogee);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_mavros_interface");
    ros::NodeHandle nh("drone_mavros_interface");

    // Subscribe to both mavros pose and twist and merge them
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/mavros/local_position/pose", 1);
    message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(nh, "/mavros/local_position/velocity_body", 1);
    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sync(pose_sub, vel_sub,10);
    sync.registerCallback(boost::bind(&publishConvertedState, _1, _2));

    // Subscribe to drone control
    ros::Subscriber drone_control_sub = nh.subscribe("/drone_control", 1, updateCurrentControl);

    // Subscribe to drone control
    ros::Subscriber target_apogee_sub = nh.subscribe("/target_apogee", 1, targetCallback);

    // Create control publisher
    pixhawk_control_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

    target_apogee_pub = nh.advertise<mavros_msgs::ActuatorControl>("/target_apogee", 10);


    // Subscribe to drone control
    ros::Subscriber rc_sub = nh.subscribe("/mavros/manual_control/control", 1, RCCallback);

    // Create state publisher
    pixhawk_state_pub = nh.advertise<drone_gnc::DroneState>("/pixhawk_drone_state", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, mavrosStateCallback);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    ros::Rate rate(200.0);
    for(int i = 100; ros::ok() && i > 0; --i){
        pixhawk_controls.at(0) = 0.6;
        pixhawk_controls.at(1) = 0.6;
        pixhawk_controls.at(2) = 0.6;
        pixhawk_controls.at(3) = 0.6;

        mavros_msgs::ActuatorControl pixhawk_control_msg;
        pixhawk_control_msg.header.stamp = ros::Time::now();
        pixhawk_control_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_MANUAL_PASSTHROUGH;
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
        pixhawk_control_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_MANUAL_PASSTHROUGH;
        pixhawk_control_msg.controls = pixhawk_controls;

        pixhawk_control_pub.publish(pixhawk_control_msg);

        ros::spinOnce();
        rate.sleep();
    }
}
