/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_ROCKET_MODEL_HPP
#define SRC_ROCKET_MODEL_HPP

#include <Eigen/Eigen>
#include <fstream>
#include <ros/ros.h>

class Rocket {
public:
    double dry_mass;
    double dry_mass_inv;

    double total_CM; // Current Cm of rocket, in real time

    std::vector<double> dry_Inertia{0, 0, 0};

    Eigen::Vector<double, 3> gravity_vector;
    Eigen::Vector<double, 3> inertia;
    Eigen::Vector<double, 3> inertia_inv;

//    Eigen::Vector<double, 3> euler_coefs;

    Rocket(ros::NodeHandle n) {
        if (n.getParam("/rocket/dry_mass", dry_mass) &&
            n.getParam("/rocket/dry_I", dry_Inertia)) {}
        else {
            ROS_ERROR("Failed to get rocket parameters");
        }

        inertia << dry_Inertia[0], dry_Inertia[1], dry_Inertia[2];
        inertia_inv = inertia.cwiseInverse();
        dry_mass_inv = 1 / dry_mass;
        // Force in inertial frame: gravity
        double g0 = 9.81;
        gravity_vector << 0, 0, -g0 * dry_mass;

//        euler_coefs << (inertia[2] - inertia[1]) / inertia[0],
//                (inertia[0] - inertia[2]) / inertia[1],
//                (inertia[1] - inertia[0]) / inertia[2];
    }


    template<typename T, typename state>
    inline void generic_rocket_dynamics(const Eigen::Matrix<T, 13, 1> x,
                                        const Eigen::Matrix<T, 3, 1> thrust_vector,
                                        const Eigen::Matrix<T, 3, 1> body_torque,
                                        const Eigen::Matrix<T, 3, 1> disturbance_force,
                                        const Eigen::Matrix<T, 3, 1> disturbance_torque,
                                        state &xdot) const noexcept {
        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));

        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Total force in inertial frame [N]
        Eigen::Vector<T, 3> total_force;
        total_force = rot_matrix * thrust_vector + gravity_vector.template cast<T>() + disturbance_force;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat((T) 0.0, x(10), x(11), x(12));

        // thrust vector torque in body frame (M = r x F)
        Eigen::Vector<T, 3> rocket_torque;
        rocket_torque << thrust_vector(1) * total_CM,
                -thrust_vector(0) * total_CM,
                (T) 0;

        // -------------- Differential equations ---------------------

        // Position derivative is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed derivative is Force/mass
        // using mass inverse to avoid computing slow division (maybe not needed?)
        xdot.segment(3, 3) = total_force * (T) dry_mass_inv;

        // Quaternion derivative is 0.5*q◦w (if w in body frame)
        xdot.segment(6, 4) = (T) 0.5 * (attitude * omega_quat).coeffs();

        Eigen::Vector<T, 3> omega = x.segment(10, 3);

        // Total torque in body frame
        Eigen::Matrix<T, 3, 1> total_torque;
        total_torque = rocket_torque + body_torque + rot_matrix.transpose() * disturbance_torque;

        // Angular speed derivative is given by Euler's rotation equations (if in body frame)
        xdot.segment(10, 3) = (total_torque - omega.cross(inertia.template cast<T>().cwiseProduct(omega)))
                .cwiseProduct(inertia_inv.template cast<T>());
    }

};

#endif //SRC_ROCKET_MODEL_HPP
