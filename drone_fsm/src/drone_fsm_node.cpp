/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "ros/ros.h"
#include "rocket_utils/FSM.h"
#include "drone_gnc/DroneExtendedState.h"
#include <time.h>

#include <sstream>
#include <string>

#include "std_msgs/String.h"

// global variable with time and state machine
rocket_utils::FSM current_fsm;
double time_zero;
drone_gnc::DroneExtendedState current_state;
bool received_state = false;
geometry_msgs::Vector3 target_apogee;

void stateCallback(const drone_gnc::DroneExtendedState::ConstPtr &rocket_state) {
//    const std::lock_guard<std::mutex> lock(state_mutex);
    current_state = *rocket_state;
    received_state = true;
}

void targetCallback(const geometry_msgs::Vector3::ConstPtr &target) {
//    const std::lock_guard<std::mutex> lock(target_mutex);
    target_apogee = *target;
}


void processCommand(const std_msgs::String &command) {
    if (command.data == "stop" || command.data == "Stop") {
        current_fsm.state_machine = rocket_utils::FSM::STOP;
    } else if (current_fsm.state_machine == rocket_utils::FSM::IDLE) {
        //received launch command
        time_zero = ros::Time::now().toSec();
        current_fsm.state_machine = rocket_utils::FSM::ASCENT;
    }
}

ros::Publisher timer_pub;

float rail_length = 0;

int main(int argc, char **argv) {

    // Init ROS time keeper node
    ros::init(argc, argv, "drone_fsm");
    ros::NodeHandle nh("drone_fsm");

    // Initialize fsm
    std::string initial_state;
    nh.param<std::string>("initial_state", initial_state, "Idle");
    if (initial_state == "Idle") current_fsm.state_machine = rocket_utils::FSM::IDLE;
    else if (initial_state == "Ascent") current_fsm.state_machine = rocket_utils::FSM::ASCENT;
    bool land_after_apogee;
    nh.param<bool>("land_after_apogee", land_after_apogee, false);

    // Create timer publisher and associated thread (100Hz)
    timer_pub = nh.advertise<rocket_utils::FSM>("/gnc_fsm_pub", 10);

    // Subscribe to commands
    ros::Subscriber command_sub = nh.subscribe("/commands", 10, processCommand);

    // Subscribe to commands
    ros::Subscriber target_sub = nh.subscribe("/target_apogee", 1, targetCallback);

    ros::Publisher target_pub = nh.advertise<geometry_msgs::Vector3>("/target_apogee", 10);

    std::vector<double> initial_target_apogee;
    if (nh.getParam("/guidance/target_apogee", initial_target_apogee) ||
        nh.getParam("/control/target_apogee", initial_target_apogee)) {
        target_apogee.x = initial_target_apogee.at(0);
        target_apogee.y = initial_target_apogee.at(1);
        target_apogee.z = initial_target_apogee.at(2);
    }


    // Subscribe to commands
    ros::Subscriber state_sub = nh.subscribe("/drone_state", 1, stateCallback);

    timer_pub.publish(current_fsm);

    nh.getParam("/environment/rail_length", rail_length);

    ros::Rate loop_rate(200);

    while (ros::ok()) {
        ros::spinOnce();

        // Update FSM
        if (current_fsm.state_machine == rocket_utils::FSM::IDLE) {

        } else if (current_fsm.state_machine == rocket_utils::FSM::ASCENT) {
            if (current_state.state.pose.position.z > 1) {
                if (current_state.state.twist.linear.z <= 0 ||
                    current_state.state.pose.position.z >= target_apogee.z) {
                    current_fsm.state_machine = rocket_utils::FSM::DESCENT;
                }
            }

        } else if (current_fsm.state_machine == rocket_utils::FSM::DESCENT) {
            if (current_state.state.twist.linear.z >= 0 &&
                current_state.state.pose.position.z < 0.3) {
                current_fsm.state_machine = rocket_utils::FSM::STOP;
            }
        }
        // Publish time + state machine
        timer_pub.publish(current_fsm);
    }

    loop_rate.sleep();
}
