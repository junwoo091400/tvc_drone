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
#include "rocket_utils/ExtendedState.h"
#include "drone_optimal_control/DroneWaypointStamped.h"
#include "drone_optimal_control/Waypoint.h"
#include "rocket_utils/Trajectory.h"
#include "drone_optimal_control/DroneTrajectory.h"

#include "rocket_utils/DroneGimbalControl.h"
#include "geometry_msgs/Vector3.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#include <time.h>

#include <iostream>
#include "drone_guidance.h"
#include "load_guidance_settings.hpp"
#include "load_drone_props.hpp"

class DroneGuidanceNode {
public:
    DroneGuidanceNode(ros::NodeHandle &nh, Drone *drone);

    void initTopics(ros::NodeHandle &nh);

    void run();

    void simulationStateCallback(const rocket_utils::State::ConstPtr &rocket_state);

    void stateCallback(const drone_optimal_control::DroneExtendedState::ConstPtr &rocket_state);

    void setPointCallback(const rocket_utils::State::ConstPtr &set_point_msg);

    void fsmCallback(const rocket_utils::FSM::ConstPtr &fsm);

    void computeTrajectory();

    void publishTrajectory();

    void publishDebugInfo();

    void startDescent();


private:
    Drone *drone;

    GuidanceSettings<double> guidance_settings;
    DroneGuidance drone_guidance;

    bool received_state = false;
    drone_optimal_control::DroneExtendedState current_state;
    rocket_utils::FSM current_fsm;
    Drone::state target_state;
    Drone::control target_control;
    Drone::state x0;
    bool started_descent = false;
    double descent_trigger_time;
    double last_computation_time;

    // Subscribers
    ros::Subscriber state_sub;
    ros::Subscriber target_sub;
    ros::Subscriber fsm_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;

    // Debug
    ros::Publisher sqp_iter_pub;
    ros::Publisher qp_iter_pub;
    ros::Publisher horizon_pub;
    ros::Publisher computation_time_pub;

    // Variables to track performance over whole simulation
    ros::Time time_compute_start;
};
