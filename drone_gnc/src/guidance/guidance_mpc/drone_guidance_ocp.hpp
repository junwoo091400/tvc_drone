/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_DRONE_GUIDANCE_OCP_HPP
#define SRC_DRONE_GUIDANCE_OCP_HPP

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
#include "drone_model.hpp"

using namespace Eigen;
using namespace std;

#define POLY_ORDER 7
#define NUM_SEG    2

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ DroneGuidanceOCP, /*NX*/ 6, /*NU*/ 3, /*NP*/ 1, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)

class DroneGuidanceOCP : public ContinuousOCP<DroneGuidanceOCP, Approximation, SPARSE> {
public:
    ~DroneGuidanceOCP() = default;

    Matrix<scalar_t, NX, 1> xs;

    shared_ptr<Drone> drone;

    scalar_t min_z;
    scalar_t min_dz;
    scalar_t max_dz;

    scalar_t horizontal_slack, max_attitude_angle, descent_min_propeller_speed;

    void init(ros::NodeHandle nh, shared_ptr<Drone> drone_ptr) {
        drone = drone_ptr;
        scalar_t max_attitude_angle_degree;
        if (nh.getParam("mpc/min_z", min_z) &&
            nh.getParam("mpc/min_dz", min_dz) &&
            nh.getParam("mpc/max_dz", max_dz) &&
            nh.getParam("mpc/horizontal_slack", horizontal_slack) &&
            nh.getParam("mpc/max_attitude_angle", max_attitude_angle_degree) &&
            nh.getParam("mpc/descent_min_propeller_speed", descent_min_propeller_speed)) {
            max_attitude_angle = max_attitude_angle_degree * M_PI / 180.0;
        } else {
            ROS_ERROR("Failed to get Guidance MPC parameter");
        }

    }

    template<typename T>
    inline void dynamics_impl(const Ref<const state_t <T>> x,
                              const Ref<const control_t <T>> u,
                              const Ref<const parameter_t <T>> p,
                              const Ref<const static_parameter_t> &d,
                              const T &t, Ref<state_t < T>>

    xdot)  const noexcept{
        T prop_av = u(0);
        T phi = u(1);
        T theta = u(2);

        T thrust = drone->getThrust(prop_av) * drone->thrust_scaling;

        Eigen::Matrix<T, 3, 1> thrust_vector;
        thrust_vector << thrust * sin(phi) * sin(theta),
                -thrust * cos(phi) * sin(theta),
                thrust * cos(theta);

        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T) 0, (T) 0, (T) - 9.81;

        xdot.head(3) = x.segment(3, 3);
        xdot.segment(3, 3) = thrust_vector * (T) drone->dry_mass_inv + gravity;

        //minimal time
        xdot *= p(0);
    }

    template<typename T>
    EIGEN_STRONG_INLINE void
    inequality_constraints_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, Ref<constraint_t < T>>

    g) const noexcept
    {
    }

    template<typename T>
    inline void lagrange_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                   const Ref<const parameter_t <T>> p,
                                   const Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        lagrange = p(0) * u(0) * u(0);
    }

    template<typename T>
    inline void mayer_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        mayer = horizontal_slack * ((x(0) - xs(0)) * (x(0) - xs(0)) + (x(1) - xs(1)) * (x(1) - xs(1)));
    }
};

#endif //SRC_DRONE_GUIDANCE_OCP_HPP
