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

template <typename scalar>
struct GuidanceSettings
{
  scalar min_z;
  scalar min_dz;
  scalar max_dz;
  scalar horizontal_slack, max_attitude_angle, descent_min_propeller_speed;
  scalar max_horizon_length;

  int max_sqp_iter, max_qp_iter, max_line_search_iter;
  std::vector<scalar> target_apogee_vec;
  std::vector<scalar> target_land_vec;

  GuidanceSettings() : target_apogee_vec(3, 0.0), target_land_vec(3, 0.0)
  {
  }
};