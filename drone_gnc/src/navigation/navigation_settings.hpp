/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

#include <vector>

template<typename scalar>
struct NavigationSettings {
    std::vector<double> initial_state;
    bool use_gps;
    bool init_estimated_params;
    bool estimate_params;

    scalar initial_thrust_scaling, initial_torque_scaling;

    double x_var, dx_var, att_var, datt_var, thrust_scaling_var, torque_scaling_var, disturbance_force_var, disturbance_force_z_var, disturbance_torque_var, disturbance_torque_z_var;
    double x_optitrack_var, pixhawk_var, pixhawk_vel_var;

    double period;

    NavigationSettings() : initial_state(13, 0.0) {
        initial_thrust_scaling = 0;
        initial_torque_scaling = 0;
    }

};