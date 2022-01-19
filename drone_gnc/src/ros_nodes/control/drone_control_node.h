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

#include <mutex>
#include <iostream>
#include <chrono>
#include <numeric>
#include <drone_mpc.h>
#include "load_mpc_settings.hpp"
#include "load_drone_props.hpp"

class DroneControlNode {
public:
    double period;

    DroneControlNode(ros::NodeHandle &nh, Drone *drone_ptr);

    void initTopics(ros::NodeHandle &nh);

    void run();

    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm);

    // Callback function to store last received state
    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state);

    // Callback function to store last received state
    void targetCallback(const geometry_msgs::Vector3 &target);

    void computeControl();

    void publishControl();

    void publishTrajectory();

    void fetchNewTarget();

    void targetTrajectoryCallback(const drone_gnc::DroneTrajectory::ConstPtr &target);

    void sampleTargetTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_state_traj,
                                Matrix<double, Drone::NU, DroneMPC::num_nodes> &mpc_target_control_traj);

    void sampleTargetTrajectoryLinear(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_state_traj,
                                      Matrix<double, Drone::NU, DroneMPC::num_nodes> &mpc_target_control_traj);

    void publishDebugInfo();

private:
    Drone *drone;

    DroneMPC drone_mpc;
    ControlMPCSettings<double> mpc_settings;

    bool received_state = false;
    drone_gnc::DroneState current_state;
    geometry_msgs::Vector3 target_apogee;
    double time_compute_start;
    bool track_guidance;

    drone_gnc::FSM current_fsm;
    double emergency_stop = false;

    ros::Timer fsm_update_thread;

    //guidance trajectory variables
    int GUIDANCE_POLY_ORDER;
    int GUIDANCE_NUM_SEG;
    int GUIDANCE_NUM_NODE;
    MatrixXd guidance_state_trajectory;
    MatrixXd guidance_control_trajectory;
    MatrixXd m_basis;
    bool received_trajectory = false;
    double guidance_t0;
    double guidance_tf;
    double start_time = 0;
    double computation_time = 0;

    double SEG_LENGTH;

    ros::Subscriber drone_state_sub;
    ros::Subscriber target_sub;
    ros::Subscriber target_traj_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;
    ros::Publisher drone_control_pub;
    ros::Subscriber fsm_sub;

    // Debug
    ros::Publisher sqp_iter_pub;
    ros::Publisher qp_iter_pub;
    ros::Publisher horizon_pub;
    ros::Publisher computation_time_pub;
};
