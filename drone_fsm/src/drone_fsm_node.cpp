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
#include <sstream>
#include <string>
#include "rocket_utils/ExtendedState.h"
#include "rocket_types/rocket_types.h"
#include "rocket_types/ros_conversions.h"

#include "std_msgs/String.h"

using namespace rocket;

class FsmNode {
private:
    RocketFSMState current_fsm;

    ros::Subscriber command_sub;
    ros::Subscriber guidance_set_point_sub;
    ros::Subscriber state_sub;
    ros::Subscriber extended_state_sub;

    ros::Publisher fsm_pub;

    double time_zero;
    RocketState current_state;
    bool is_calibration_done = false;
    RocketState target_set_point = RocketState::Zero();

    bool land_after_apogee;

public:
    FsmNode() : current_fsm(RocketFSMState::IDLE) {
        ros::NodeHandle nh("~");

        time_zero = ros::Time::now().toSec();

        nh.param<bool>("land_after_apogee", land_after_apogee, false);

        // Create timer publisher and associated thread (100Hz)
        fsm_pub = nh.advertise<rocket_utils::FSM>("/gnc_fsm_pub", 10);

        command_sub = nh.subscribe("/commands", 10, &FsmNode::processCommand, this);
        guidance_set_point_sub = nh.subscribe("/guidance/set_point", 1, &FsmNode::guidanceSetPointCallback, this);

        std::vector<double> initial_target_apogee;
        if (nh.getParam("/guidance/target_apogee", initial_target_apogee) ||
            nh.getParam("/control/target_apogee", initial_target_apogee)) {
          target_set_point.position.x = initial_target_apogee.at(0);
          target_set_point.position.y = initial_target_apogee.at(1);
          target_set_point.position.z = initial_target_apogee.at(2);
        }

        // Subscribe to commands
        extended_state_sub = nh.subscribe("/extended_kalman_rocket_state", 1, &FsmNode::extendedStateCallback, this);
    }

    void run() {
        switch (current_fsm) {
            case CALIBRATION: {
                if (is_calibration_done) {
                    current_fsm = RocketFSMState::IDLE;
                }
                break;
            }
            case IDLE: {
                // Do nothing
                break;
            }
            case ASCENT: {
                if (land_after_apogee &&
                    current_state.position.z > 1 &&
                    (current_state.velocity.z <= 0 || current_state.position.z >= target_set_point.position.z)) {
                    current_fsm = RocketFSMState::LANDING;
                }
                break;
            }
            case LANDING: {
                if (current_state.velocity.z >= 0 &&
                    current_state.position.z < 0.3) {
                    current_fsm = RocketFSMState::STOP;
                }
                break;
            }
            case STOP: {
                // Do nothing
                break;
            }
            default:
                throw std::runtime_error("Invalid FSM");
        }

        // Publish the current state machine
        rocket_utils::FSM fsm_msg = toROS(current_fsm);
        fsm_msg.header.stamp = ros::Time::now();
        fsm_msg.launch_time = ros::Time(time_zero);
        fsm_pub.publish(fsm_msg);
    }
private:
    void extendedStateCallback(const rocket_utils::ExtendedState::ConstPtr &extended_rocket_state) {
        current_state = fromROS(extended_rocket_state->state);
        is_calibration_done = extended_rocket_state->is_calibrated;
    }

    void guidanceSetPointCallback(const rocket_utils::State::ConstPtr &set_point_msg) {
      target_set_point = fromROS(*set_point_msg);
    }

    void processCommand(const std_msgs::String &command) {
        if (command.data == "stop" || command.data == "Stop") {
            current_fsm = RocketFSMState::STOP;
        } else if (current_fsm == RocketFSMState::IDLE) {
            //received launch command
            time_zero = ros::Time::now().toSec();
            current_fsm = RocketFSMState::ASCENT;
        }
    }
};

int main(int argc, char **argv) {
    // Init ROS FSM node
    ros::init(argc, argv, "drone_fsm");

    FsmNode fsm_node;

    ros::Rate loop_rate(200);

    while (ros::ok()) {
        ros::spinOnce();
        fsm_node.run();

        loop_rate.sleep();
    }
}
