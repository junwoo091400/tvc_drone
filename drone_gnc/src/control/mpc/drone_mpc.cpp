/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "drone_mpc.h"

DroneMPC::DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone(drone_ptr) {
    // Initialize rocket class with useful parameters
    ocp().init(nh, drone);
    int max_sqp_iter, max_qp_iter, max_line_search_iter;
    if (nh.getParam("mpc/max_sqp_iter", max_sqp_iter) &&
        nh.getParam("mpc/max_line_search_iter", max_line_search_iter) &&
        nh.getParam("mpc/period", period) &&
        nh.getParam("mpc/max_qp_iter", max_qp_iter) &&
        nh.getParam("mpc/horizon_length", max_horizon_length)) {

        settings().max_iter = max_sqp_iter;
        settings().line_search_max_iter = max_line_search_iter;
        qp_settings().max_iter = max_qp_iter;
    } else {
        ROS_ERROR("Failed to get MPC parameters");
    }

    set_time_limits(0, 1);

    // Setup constraints
    const double inf = std::numeric_limits<double>::infinity();
    const double eps = 1e-3;

    ocp_control lbu, ubu;
    lbu << -drone->max_servo_rate, -drone->max_servo_rate,
            drone->min_propeller_speed, -drone->max_propeller_delta / 2; // lower bound on control
    ubu << drone->max_servo_rate, drone->max_servo_rate,
            drone->max_propeller_speed, drone->max_propeller_delta / 2; // upper bound on control

    ocp_state lbx, ubx;
    lbx << -inf, -inf, ocp().min_z - eps,
            -ocp().max_dx, -ocp().max_dx, ocp().min_dz,
            -inf, -inf, -inf, -inf,
            -ocp().max_datt, -ocp().max_datt, -inf,
            -drone->max_servo1_angle, -drone->max_servo2_angle;
    ubx << inf, inf, inf,
            ocp().max_dx, ocp().max_dx, ocp().max_dz,
            inf, inf, inf, inf,
            ocp().max_datt, ocp().max_datt, inf,
            drone->max_servo1_angle, drone->max_servo2_angle;

    ocp_constraint ubg, lbg;
    //TODO fix attitude constraint [cos(maxAttitudeAngle) 1]
    lbg << cos(ocp().max_attitude_angle), drone->min_propeller_speed, drone->min_propeller_speed;
    ubg << inf, drone->max_propeller_speed, drone->max_propeller_speed;

    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);
    constraints_bounds(lbg, ubg);

    // Initial guess
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

Drone::state DroneMPC::solution_x_at(const double t) {
    double scaled_t = t / ocp().horizon_length;
    return MPC::solution_x_at(scaled_t).segment(0, 13).cwiseProduct(ocp().x_drone_unscaling_vec);
}

Drone::control DroneMPC::solution_u_at(const double t) {
    double scaled_t = t / ocp().horizon_length;
    Drone::control u;
    u.segment(0, 2) = MPC::solution_x_at(scaled_t).segment(13, 2);
    u.segment(2, 2) = MPC::solution_u_at(scaled_t).segment(2, 2);
    return u.cwiseProduct(ocp().u_drone_unscaling_vec);
}

Drone::state DroneMPC::solution_x_at(const int i) {
    return MPC::solution_x_at(i).segment(0, 13).cwiseProduct(ocp().x_drone_unscaling_vec);
}

Drone::control DroneMPC::solution_u_at(const int i) {
    Drone::control u;
    u.segment(0, 2) = MPC::solution_x_at(i).segment(13, 2);
    u.segment(2, 2) = MPC::solution_u_at(i).segment(2, 2);
    return u.cwiseProduct(ocp().u_drone_unscaling_vec);
}

double DroneMPC::node_time(int i) {
    return time_grid(i) * ocp().horizon_length;
}

void DroneMPC::solve(Drone::state &x0) {
    Drone::state predicted_x0;
    integrateX0(x0, predicted_x0);

    //servo rate constraint
    ocp_state previous_x = MPC::solution_x_at(0).cwiseProduct(ocp().x_unscaling_vec);
    double maxServoRate = drone->max_servo_rate;
    ocp_state lbx0;
    lbx0.segment(0, 13) = predicted_x0;
    lbx0(13) = previous_x(13) - maxServoRate * period;
    lbx0(14) = previous_x(14) - maxServoRate * period;

    ocp_state ubx0;
    ubx0.segment(0, 13) = predicted_x0;
    ubx0(13) = previous_x(13) + maxServoRate * period;
    ubx0(14) = previous_x(14) + maxServoRate * period;

    double eps = 1e-7;
    initial_conditions(lbx0.cwiseProduct(ocp().x_scaling_vec).array() - eps,
                       ubx0.cwiseProduct(ocp().x_scaling_vec).array() + eps);
    MPC::solve();
}

void DroneMPC::integrateX0(const Drone::state x0, Drone::state &new_x0) {
    Drone::control control = solution_u_at(0);
    drone->stepRK4(x0, control, period, new_x0);
}

void DroneMPC::setHorizonLength(double horizon_length) {
    ocp().horizon_length = max(1e-3, min(horizon_length, max_horizon_length));
}

void DroneMPC::setTargetStateTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> target_state_trajectory) {
    //assign while scaling
    ocp().target_state_trajectory = ocp().x_drone_scaling_vec.asDiagonal() * target_state_trajectory;
}

void DroneMPC::setTargetControlTrajectory(Matrix<double, Drone::NU, DroneMPC::num_nodes> target_control_trajectory) {
    ocp().target_control_trajectory = ocp().u_drone_scaling_vec.asDiagonal() * target_control_trajectory;
}

void DroneMPC::reset() {
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


//void DroneMPC::warmStart() {
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
//}
