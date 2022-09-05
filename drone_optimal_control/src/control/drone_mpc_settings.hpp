/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

template <typename scalar>
struct ControlMPCSettings
{
  scalar x_cost, dx_cost, z_cost, dz_cost, att_cost, datt_cost, servo_cost, thrust_cost, torque_cost, droll_cost;
  scalar weight_scaling;

  scalar max_attitude_angle, min_z, min_dz, max_dx, max_dz, max_datt, scaling_x, scaling_z;

  scalar horizon_length;

  int max_sqp_iter, max_qp_iter, max_line_search_iter;

  scalar period;
};