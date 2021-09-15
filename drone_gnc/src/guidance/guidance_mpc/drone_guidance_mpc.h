/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_DRONE_GUIDANCE_MPC_H
#define SRC_DRONE_GUIDANCE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "guidance_solver.hpp"

class DroneGuidanceMPC : private MPC<DroneGuidanceOCP, GuidanceSolver> {

public:
    using ocp_state = state_t;
    using ocp_control = control_t;
    using ocp_constraint = constraint_t;

    Drone::state target_apogee;
    Drone::state target_land;

    using MPC::num_nodes;

    DroneGuidanceMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void solve(Drone::state &x0);

    void initGuess(Drone::state &x0, Drone::state &target);

    void setTarget(Drone::state &target_state, Drone::control &target_control);

    Drone::state solution_x_at(const int t);

    Drone::control solution_u_at(const int t);

    using MPC::solution_p;

    void setDescentConstraints();

    double node_time(int i);

    void precomputeDescent();

    void warmStartDescent(Drone::state x0);

    using MPC::info;

    double max_horizon_length;

private:
    std::shared_ptr<Drone> drone;
    traj_control_t descent_control_sol;
    traj_state_t descent_state_sol;
    dual_var_t descent_dual_sol;
    parameter_t descent_p_sol;
};


#endif //SRC_DRONE_GUIDANCE_MPC_H
