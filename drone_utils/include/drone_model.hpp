/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

template<typename scalar>
struct DroneProps {
    double dry_mass;
    double dry_mass_inv;

    double total_CM; // Current Cm of rocket, in real time

    std::vector<double> dry_Inertia{0, 0, 0};

    Eigen::Vector3d inertia;
    Eigen::Vector3d inertia_inv;

    // parameters
    double min_propeller_speed, max_propeller_speed;
    double max_propeller_delta;
    double max_servo1_angle, max_servo2_angle;
    double max_servo_rate;
};


class Drone {
public:
    static const int NX = 13;
    static const int NU = 4;
    static const int NP = 8;

    template<typename T>
    using state_t = Eigen::Matrix<T, NX, 1>;
    using state = state_t<double>;
    template<typename T>
    using control_t = Eigen::Matrix<T, NU, 1>;
    using control = control_t<double>;

    const double g = 9.81;

    DroneProps<double> props;

    Drone(DroneProps<double> &props_) : props(props_) {
    }

    //thrust 2rd order model
    const double a = -0.000035075437403;
    const double b = 0.029719658777622;
    const double c = -0.341510545964088;

    // roll model: rot_acc = delta * delta_to_acc
    // torque = I * rot_acc
    const double delta_to_acc = -0.1040;

    double getAverageSpeed(const double thrust) const {
        double average = (-b + std::sqrt(b * b - 4 * a * (c - thrust / g))) / (2 * a);
        return average;
    }

    double getHoverSpeedAverage() {
        return getAverageSpeed(g * props.dry_mass);
    }


    template<typename T>
    inline T getThrust(const T prop_average) const {
        T thrust = (a * prop_average * prop_average + b * prop_average + c) * g;
        return thrust;
    }

    template<typename T>
    inline T getTorque(const T prop_delta) const {
        T torque = props.dry_Inertia[2] * prop_delta * delta_to_acc;
        return torque;
    }
};