/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_DRONE_CONTROL_OCP_HPP
#define SRC_DRONE_CONTROL_OCP_HPP

#include "ros/ros.h"

#include <string>
#include <math.h>

#include <chrono>

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/osqp_interface.hpp"
#include "control/mpc_wrapper.hpp"
#include "mpc_utils.hpp"

using namespace Eigen;

// Poly MPC stuff ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

using namespace std;

#define POLY_ORDER 6
#define NUM_SEG    1

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ DroneControlOCP, /*NX*/ 15, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/3, /*TYPE*/ double)

class DroneControlOCP : public ContinuousOCP<DroneControlOCP, Approximation, SPARSE> {
public:
    ~DroneControlOCP() = default;

    Matrix<scalar_t, 11, 1> Q;
    Matrix<scalar_t, Drone::NU, 1> R;
    Matrix<scalar_t, 11, 11> QN;

    Matrix<scalar_t, Drone::NX, NUM_NODES> target_state_trajectory;
    Matrix<scalar_t, Drone::NU, NUM_NODES> target_control_trajectory;

    shared_ptr<Drone> drone;

    scalar_t max_attitude_angle;

    scalar_t min_z;
    scalar_t min_dz;
    scalar_t max_dx;
    scalar_t max_dz;
    scalar_t max_datt;
    scalar_t scaling_x;
    scalar_t scaling_z;

    Matrix<scalar_t, NX, 1> x_unscaling_vec;
    Matrix<scalar_t, NU, 1> u_unscaling_vec;

    Matrix<scalar_t, NX, 1> x_scaling_vec;
    Matrix<scalar_t, NU, 1> u_scaling_vec;

    Matrix<scalar_t, Drone::NX, 1> x_drone_unscaling_vec;
    Matrix<scalar_t, Drone::NU, 1> u_drone_unscaling_vec;
    Matrix<scalar_t, Drone::NX, 1> x_drone_scaling_vec;
    Matrix<scalar_t, Drone::NU, 1> u_drone_scaling_vec;

    scalar_t horizon_length;

    void init(ros::NodeHandle nh, shared_ptr<Drone> drone_ptr) {
        drone = drone_ptr;
        scalar_t x_cost, dx_cost, z_cost, dz_cost, att_cost, datt_cost, servo_cost, thrust_cost, torque_cost, droll_cost;
        scalar_t max_attitude_angle_degree, weight_scaling;
        if (nh.getParam("mpc/min_z", min_z) &&
            nh.getParam("mpc/min_dz", min_dz) &&
            nh.getParam("mpc/max_dx", max_dx) &&
            nh.getParam("mpc/max_dz", max_dz) &&
            nh.getParam("mpc/max_datt", max_datt) &&
            nh.getParam("mpc/scaling_x", scaling_x) &&
            nh.getParam("mpc/scaling_z", scaling_z) &&
            nh.getParam("mpc/weight_scaling", weight_scaling) &&

            nh.getParam("mpc/state_costs/x", x_cost) &&
            nh.getParam("mpc/state_costs/dz", dx_cost) &&
            nh.getParam("mpc/state_costs/z", z_cost) &&
            nh.getParam("mpc/state_costs/dz", dz_cost) &&
            nh.getParam("mpc/state_costs/att", att_cost) &&
            nh.getParam("mpc/state_costs/datt", datt_cost) &&
            nh.getParam("mpc/state_costs/droll", droll_cost) &&
            nh.getParam("mpc/input_costs/servo", servo_cost) &&
            nh.getParam("mpc/input_costs/thrust", thrust_cost) &&
            nh.getParam("mpc/input_costs/torque", torque_cost) &&

            nh.getParam("mpc/max_attitude_angle", max_attitude_angle_degree) &&

            nh.getParam("mpc/horizon_length", horizon_length)) {

            Q << x_cost, x_cost, z_cost,
                    dx_cost, dx_cost, dz_cost,
                    att_cost, att_cost,
                    datt_cost, datt_cost, droll_cost;

            R << servo_cost, servo_cost, thrust_cost, torque_cost;

            computeLQRTerminalCost(drone, Q, R, QN);

            ROS_INFO_STREAM("QN" << QN);

            max_attitude_angle = max_attitude_angle_degree * (M_PI / 180);

            x_unscaling_vec << scaling_x, scaling_x, scaling_z,
                    max_dx, max_dx, max_dz,
                    1, 1, 1, 1,
                    max_datt, max_datt, max_datt,
                    drone->max_servo1_angle, drone->max_servo2_angle;
            x_scaling_vec = x_unscaling_vec.cwiseInverse();

            u_unscaling_vec << drone->max_servo_rate, drone->max_servo_rate,
                    drone->max_propeller_speed, drone->max_propeller_delta / 2;
            u_scaling_vec = u_unscaling_vec.cwiseInverse();

//            u_unscaling_vec.setOnes();
//            u_scaling_vec.setOnes();
//            x_unscaling_vec.setOnes();
//            x_scaling_vec.setOnes();

            //scale costs
            x_drone_unscaling_vec = x_unscaling_vec.segment(0, Drone::NX);
            u_drone_unscaling_vec.segment(0, 2) = x_unscaling_vec.segment(Drone::NX, 2);
            u_drone_unscaling_vec.segment(2, 2) = u_unscaling_vec.segment(2, 2);

            x_drone_scaling_vec = x_drone_unscaling_vec.cwiseInverse();
            u_drone_scaling_vec = u_drone_unscaling_vec.cwiseInverse();

            Matrix<scalar_t, 11, 1> Q_unscaling_vec;
            Q_unscaling_vec.segment(0, 8) = x_unscaling_vec.segment(0, 8);
            Q_unscaling_vec.segment(8, 3) = x_unscaling_vec.segment(10, 3);
            Q = Q.cwiseProduct(Q_unscaling_vec).cwiseProduct(Q_unscaling_vec);

            Matrix<scalar_t, 11, 11> Q_unscaling_mat;
            Q_unscaling_mat.setZero();
            Q_unscaling_mat.diagonal() << Q_unscaling_vec;
            QN = Q_unscaling_mat * QN * Q_unscaling_mat;

            R = R.cwiseProduct(u_drone_unscaling_vec).cwiseProduct(u_drone_unscaling_vec);
            ROS_INFO_STREAM("u_drone_scale" << u_drone_unscaling_vec);

            ROS_INFO_STREAM("Q" << Q);
            ROS_INFO_STREAM("R" << R);
            ROS_INFO_STREAM("QN" << QN);
            R /= weight_scaling;
            Q /= weight_scaling;
            QN /= weight_scaling;
            R = R.eval();
            Q = Q.eval();
            QN = QN.eval();
        } else {
            ROS_ERROR("Failed to get MPC parameter");
        }

    }

    void getConstraints(state_t<scalar_t> &lbx, state_t<scalar_t> &ubx,
                        control_t<scalar_t> &lbu, control_t<scalar_t> &ubu,
                        constraint_t <scalar_t> &lbg, constraint_t <scalar_t> &ubg) {
        const double inf = std::numeric_limits<double>::infinity();
        const double eps = 1e-3;

        lbu << -drone->max_servo_rate, -drone->max_servo_rate,
                drone->min_propeller_speed, -drone->max_propeller_delta / 2; // lower bound on control
        ubu << drone->max_servo_rate, drone->max_servo_rate,
                drone->max_propeller_speed, drone->max_propeller_delta / 2; // upper bound on control

        lbx << -inf, -inf, min_z - eps,
                -max_dx, -max_dx, min_dz,
                -inf, -inf, -inf, -inf,
                -max_datt, -max_datt, -inf,
                -drone->max_servo1_angle, -drone->max_servo2_angle;

        ubx << inf, inf, inf,
                max_dx, max_dx, max_dz,
                inf, inf, inf, inf,
                max_datt, max_datt, inf,
                drone->max_servo1_angle, drone->max_servo2_angle;

        //TODO fix attitude constraint [cos(maxAttitudeAngle) 1]
        lbg << cos(max_attitude_angle), drone->min_propeller_speed, drone->min_propeller_speed;
        ubg << inf, drone->max_propeller_speed, drone->max_propeller_speed;
        ROS_INFO_STREAM(lbg);
        ROS_INFO_STREAM("min cos" << cos(max_attitude_angle));
//
        lbu = lbu.cwiseProduct(u_scaling_vec);
        ubu = ubu.cwiseProduct(u_scaling_vec);
        ROS_INFO_STREAM("lbu" << lbu.transpose());
        ROS_INFO_STREAM("ubu" << ubu.transpose());

        lbx = lbx.cwiseProduct(x_scaling_vec);
        ubx = ubx.cwiseProduct(x_scaling_vec);
        ROS_INFO_STREAM("lbx" << lbx.transpose());
        ROS_INFO_STREAM("ubx" << ubx.transpose());

    }

    template<typename T>
    inline void dynamics_impl(const Ref<const state_t<T>> x,
                              const Ref<const control_t<T>> u,
                              const Ref<const parameter_t <T>> p,
                              const Ref<const static_parameter_t> &d,
                              const T &t, Ref<state_t<T>>

                              xdot) const noexcept {
        Matrix<T, NX, 1> x_unscaled = x.cwiseProduct(x_unscaling_vec.template cast<T>());
        Matrix<T, NU, 1> u_unscaled = u.cwiseProduct(u_unscaling_vec.template cast<T>());

        Matrix<T, Drone::NX, 1> x_drone = x_unscaled.segment(0, Drone::NX);

        Matrix<T, Drone::NU, 1> u_drone;
        u_drone.segment(0, 2) = x_unscaled.segment(Drone::NX, 2);
        u_drone.segment(2, 2) = u_unscaled.segment(2, 2);

        Matrix<T, Drone::NP, 1> params;

        drone->getParams(params);

        drone->state_dynamics(x_drone, u_drone, params, xdot);
        xdot.segment(13, 2) = u_unscaled.segment(0, 2);

        //unscale
        xdot = xdot.cwiseProduct(x_scaling_vec.template cast<T>());

        xdot *= (T) horizon_length;

    }

    template<typename T>
    EIGEN_STRONG_INLINE void
    inequality_constraints_impl(const Ref<const state_t<T>> x, const Ref<const control_t<T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, Ref<constraint_t < T>>

    g) const noexcept
    {
        Matrix<T, 2, 1> u_drone = u.segment(2, 2).cwiseProduct(u_unscaling_vec.segment(2, 2).template cast<T>());


        g(0) = x(9) * x(9) - x(6) * x(6) - x(7) * x(7) + x(8) * x(8);
        g(1) = u_drone(0) + 0.5 * u_drone(1);
        g(2) = u_drone(0) - 0.5 * u_drone(1);
    }

    //workaround to get the current node index in cost functions
    int k = NUM_NODES - 1;

    template<typename T>
    inline void lagrange_term_impl(const Ref<const state_t<T>> x, const Ref<const control_t<T>> u,
                                   const Ref<const parameter_t <T>> p,
                                   const Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        Matrix<T, 13, 1> x_error = x.segment(0, 13) - target_state_trajectory.col(k).template cast<T>();
        Matrix<T, 11, 1> x_error2;
        x_error2.segment(0, 6) = x_error.segment(0, 6);
        x_error2(6) = x_error(9) * x_error(6) - x_error(7) * x_error(8);
        x_error2(7) = x_error(9) * x_error(7) + x_error(6) * x_error(8);
        x_error2.segment(8, 3) = x_error.segment(10, 3);

        Matrix<T, 4, 1> u_drone;
        u_drone.segment(0, 2) = x.segment(13, 2);
        u_drone.segment(2, 2) = u.segment(2, 2);
        Matrix<T, NU, 1> u_error = u_drone - target_control_trajectory.col(k).template cast<T>();


        lagrange = (x_error2.dot(Q.template cast<T>().cwiseProduct(x_error2)) +
                    u_error.dot(R.template cast<T>().cwiseProduct(u_error))) * horizon_length;

        k--;
    }

    template<typename T>
    inline void mayer_term_impl(const Ref<const state_t<T>> x, const Ref<const control_t<T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        k = NUM_NODES - 1;

        Matrix<T, 13, 1> x_error = x.segment(0, 13) - target_state_trajectory.col(k).template cast<T>();
        Matrix<T, 11, 1> x_error2;
        x_error2.segment(0, 6) = x_error.segment(0, 6);
        x_error2(6) = x_error(9) * x_error(6) - x_error(7) * x_error(8);
        x_error2(7) = x_error(9) * x_error(7) + x_error(6) * x_error(8);
        x_error2.segment(8, 3) = x_error.segment(10, 3);

        mayer = x_error2.dot(QN.template cast<T>() * x_error2);
//        mayer = (T) 0;
    }
};

#endif //SRC_DRONE_CONTROL_OCP_HPP
