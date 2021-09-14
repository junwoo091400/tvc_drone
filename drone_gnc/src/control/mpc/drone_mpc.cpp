/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This source code is subject to the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. If a copy of the GNU General Public License was not distributed
 * with this file, you can obtain one at http://www.gnu.org/licenses/.
 */

#include "drone_mpc.h"

DroneMPC::DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone(drone_ptr), solution_time(0) {
    // Initialize rocket class with useful parameters
    ocp().init(nh, drone);
    int max_sqp_iter, max_qp_iter, max_line_search_iter;
    nh.getParam("mpc/max_sqp_iter", max_sqp_iter);
    nh.getParam("mpc/max_line_search_iter", max_line_search_iter);
    nh.getParam("mpc/mpc_period", mpc_period);
    nh.getParam("mpc/feedforward_period", feedforward_period);
    nh.getParam("mpc/max_qp_iter", max_qp_iter);
    nh.param("/is_simu", is_simu, true);

    settings().max_iter = max_sqp_iter;
    settings().line_search_max_iter = max_line_search_iter;
    qp_settings().max_iter = max_qp_iter;

    nh.getParam("mpc/horizon_length", max_horizon_length);
    set_time_limits(0, 1);

    // Setup constraints
    ocp_state lbx, ubx;
    ocp_control lbu, ubu;
    ocp_constraint ubg, lbg;
    ocp().getConstraints(lbx, ubx, lbu, ubu, lbg, ubg);
    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);
    constraints_bounds(lbg, ubg);

    // Initial state
    ocp_state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0,
            0, 0;
    x_guess(x0.cwiseProduct(ocp().x_scaling_vec).replicate(ocp().NUM_NODES, 1));
    ocp_control u0;
    u0 << 0, 0, drone->getHoverSpeedAverage(), 0;
    u_guess(u0.cwiseProduct(ocp().u_scaling_vec).replicate(ocp().NUM_NODES, 1));

    Drone::control target_control;
    target_control << 0, 0, drone->getHoverSpeedAverage(), 0;
    Drone::state target_state;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    init_time = ros::Time::now().toSec();
    fixed_computation_time = mpc_period * 0.93;
}


drone_gnc::DroneControl DroneMPC::getControlMessage(double t) {
    t = std::max(std::min(t, ocp().horizon_length), 0.0);
    Drone::control interpolated_control = solution_u_at(t);

    drone_gnc::DroneControl drone_control;
    drone_control.servo1 = interpolated_control(0);
    drone_control.servo2 = interpolated_control(1);
    drone_control.bottom = interpolated_control(2) - interpolated_control(3) / 2;
    drone_control.top = interpolated_control(2) + interpolated_control(3) / 2;

    drone_control.servo1 = std::min(std::max(drone_control.servo1, -drone->max_servo1_angle),
                                    drone->max_servo1_angle);
    drone_control.servo2 = std::min(std::max(drone_control.servo2, -drone->max_servo2_angle),
                                    drone->max_servo2_angle);
    drone_control.top = std::min(std::max(drone_control.top, drone->min_propeller_speed),
                                 drone->max_propeller_speed);
    drone_control.bottom = std::min(std::max(drone_control.bottom, drone->min_propeller_speed),
                                    drone->max_propeller_speed);
    drone_control.header.stamp = ros::Time::now();
    return drone_control;
}

void DroneMPC::warmStart() {
//    //warm start
//    traj_state_t x_guess;
//    traj_control_t u_guess;
//    int NX = ocp().NX;
//    int NU = ocp().NU;
//    int NUM_NODES = ocp().NUM_NODES;
//    state x_interp;
//    control u_interp;
//    double previous_interp_time = 0;
//
//    double time_since_last_solve = feedforward_period;
//
//    for (int i = 0; i < NUM_NODES; i++) {
//        double interp_time = time_grid(i) + time_since_last_solve;
//        if (interp_time <= horizon_length) {
//            x_interp = solution_x_at(interp_time);
//            u_interp = solution_u_at(interp_time);
//        } else {
////            drone->stepRK4(x_interp.cwiseProduct(mpc.ocp().x_unscaling_vec),
////                           u_interp.cwiseProduct(mpc.ocp().u_unscaling_vec),
////                           interp_time - previous_interp_time,
////                           x_interp);
////            x_interp = x_interp.cwiseProduct(mpc.ocp().x_scaling_vec);
//        }
//        x_guess.segment((NUM_NODES - i - 1) * NX, NX) = x_interp;
//        u_guess.segment((NUM_NODES - i - 1) * NU, NU) = u_interp;
//        previous_interp_time = interp_time;
//    }
//    x_guess(x_guess);
//    u_guess(u_guess);
}

Drone::state DroneMPC::solution_x_at(const double t) {
    return MPC::solution_x_at(t).segment(0, 13).cwiseProduct(ocp().x_drone_unscaling_vec);
}

Drone::control DroneMPC::solution_u_at(const double t) {
    Drone::control u;
    u.segment(0, 2) = MPC::solution_x_at(t).segment(13, 2);
    u.segment(2, 2) = MPC::solution_u_at(t).segment(2, 2);
    return u.cwiseProduct(ocp().u_drone_unscaling_vec);
}

Drone::state DroneMPC::solution_x_at(const int t) {
    return MPC::solution_x_at(t).segment(0, 13).cwiseProduct(ocp().x_drone_unscaling_vec);
}

Drone::control DroneMPC::solution_u_at(const int t) {
    Drone::control u;
    u.segment(0, 2) = MPC::solution_x_at(t).segment(13, 2);
    u.segment(2, 2) = MPC::solution_u_at(t).segment(2, 2);
    return u.cwiseProduct(ocp().u_drone_unscaling_vec);
}

double DroneMPC::node_time(int i) {
    return time_grid(i)*ocp().horizon_length;
}

void DroneMPC::solve(Drone::state &x0) {
    double computation_start_time = ros::Time::now().toSec();
    solution_time = computation_start_time + mpc_period;

    Drone::state predicted_x0;
    integrateX0(x0, predicted_x0);

    //servo rate constraint
    ocp_state previous_x = MPC::solution_x_at(0).cwiseProduct(ocp().x_unscaling_vec);
    double maxServoRate = drone->max_servo_rate;
    ocp_state lbx0;
    lbx0.segment(0, 13) = predicted_x0;
    lbx0(13) = previous_x(13) - maxServoRate * mpc_period;
    lbx0(14) = previous_x(14) - maxServoRate * mpc_period;

    ocp_state ubx0;
    ubx0.segment(0, 13) = predicted_x0;
    ubx0(13) = previous_x(13) + maxServoRate * mpc_period;
    ubx0(14) = previous_x(14) + maxServoRate * mpc_period;

    double eps = 1e-7;
    initial_conditions(lbx0.cwiseProduct(ocp().x_scaling_vec).array() - eps, ubx0.cwiseProduct(ocp().x_scaling_vec).array() + eps);

//    warmStart();

    if (is_simu) {
        // in simulation, sleep to account fo
        ros::Duration(mpc_period - feedforward_period).sleep();
    }

    double time_now = ros::Time::now().toSec();
    MPC::solve();
    last_computation_time = (ros::Time::now().toSec() - time_now) * 1000;
}

void DroneMPC::integrateX0(const Drone::state x0, Drone::state &new_x0) {
    new_x0 = x0;

    int max_ff_index = std::round(mpc_period / feedforward_period);

    int i;
    for (i = 0; i < max_ff_index - 1; i++) {
        Drone::control interpolated_control = solution_u_at(feedforward_period * i);
        drone->stepRK4(x0, interpolated_control, mpc_period, new_x0);
    }
    i++;

    Drone::control interpolated_control = solution_u_at(feedforward_period * i);
    drone->stepRK4(x0, interpolated_control, mpc_period, new_x0);
}

void DroneMPC::setHorizonLength(double horizon_length){
    ocp().horizon_length = max(1e-3, min(horizon_length, max_horizon_length));
}

void DroneMPC::setTargetStateTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> target_state_trajectory){
    //assign while scaling
    ocp().target_state_trajectory = ocp().x_drone_scaling_vec.asDiagonal() * target_state_trajectory;
}

void DroneMPC::setTargetControlTrajectory(Matrix<double, Drone::NU, DroneMPC::num_nodes> target_control_trajectory){
    ocp().target_control_trajectory = ocp().u_drone_scaling_vec.asDiagonal() * target_control_trajectory;
}
void DroneMPC::reset(){
    DroneMPC::ocp_state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    x_guess(x0.cwiseProduct(ocp().x_scaling_vec).replicate(ocp().NUM_NODES, 1));
    DroneMPC::ocp_control u0;
    u0 << 0, 0, drone->getHoverSpeedAverage(), 0;
    u_guess(u0.cwiseProduct(ocp().u_scaling_vec).replicate(ocp().NUM_NODES, 1));

    DroneMPC::dual_var_t dual;
    dual.setZero();
    lam_guess(dual);
}