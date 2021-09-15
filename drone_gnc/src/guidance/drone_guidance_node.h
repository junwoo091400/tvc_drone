/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneWaypointStamped.h"
#include "drone_gnc/Waypoint.h"
#include "drone_gnc/Trajectory.h"
#include "drone_gnc/DroneTrajectory.h"

#include "drone_gnc/DroneControl.h"
#include "geometry_msgs/Vector3.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#include "drone_gnc/GetFSM.h"

#include <time.h>

#include <iostream>
#include "guidance_mpc/drone_guidance_mpc.h"

#define USE_BACKUP_CONTROLLER false

class DroneGuidanceNode {
public:
    DroneGuidanceNode(ros::NodeHandle &nh, const std::shared_ptr<Drone> &drone_ptr);

    void initTopics(ros::NodeHandle &nh);

    void run();

    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state);

    void targetCallback(const geometry_msgs::Vector3 &target);

    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm);

    void computeTrajectory();

    void publishTrajectory();

    void publishDebugInfo();

    void startDescent();


private:
    DroneGuidanceMPC drone_mpc;
    std::shared_ptr<Drone> drone;

    bool received_state = false;
    drone_gnc::DroneState current_state;
    drone_gnc::FSM current_fsm;
    Drone::state target_state;
    Drone::control target_control;
    Drone::state x0;
    bool started_descent = false;
    double descent_trigger_time;
    double last_computation_time;

    // Subscribers
    ros::Subscriber rocket_state_sub;
    ros::Subscriber target_sub;
    ros::Subscriber fsm_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;

    // Debug
    ros::Publisher sqp_iter_pub;
    ros::Publisher qp_iter_pub;
    ros::Publisher horizon_pub;
    ros::Publisher computation_time_pub;

    ros::ServiceClient set_fsm_client;
    // Variables to track performance over whole simulation
    ros::Time time_compute_start;
};
