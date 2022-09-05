/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_DRONE_MPC_UTILS_HPP
#define SRC_DRONE_MPC_UTILS_HPP

#include <time.h>

#include <iostream>
#include <chrono>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "unsupported/Eigen/Polynomials"
#include "unsupported/Eigen/MatrixFunctions"

#include <autodiff/AutoDiffScalar.h>
#include "drone_model.hpp"
#include <memory>

static const int NX = DroneEuler::NX;
static const int NU = DroneEuler::NU;

using namespace Eigen;
using namespace std;

// Autodiff dor state
template <typename scalar_t>
using state_t = Matrix<scalar_t, NX, 1>;
using state = state_t<double>;

template <typename scalar_t>
using control_t = Matrix<scalar_t, NU, 1>;
using control = control_t<double>;

using ad_state = AutoDiffScalar<state>;
using ad_control = AutoDiffScalar<control>;

void computeLinearizedModel(DroneEuler* drone, Matrix<double, NX, NX>& A, Matrix<double, NX, NU>& B, state x_bar,
                            control u_bar);

bool solveRiccatiIterationC(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
                            const Eigen::MatrixXd& R, Eigen::MatrixXd& P, const double dt, const double& tolerance,
                            const uint iter_max);

#endif