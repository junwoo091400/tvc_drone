/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "mpc_utils.h"

void computeLinearizedModel(DroneEuler* drone, Matrix<double, NX, NX>& A, Matrix<double, NX, NU>& B, state x_bar,
                            control u_bar)
{
  DroneEuler::parameters params;
  params << 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  /** initialize derivatives */
  state_t<ad_state> ADx(x_bar);
  int div_size = ADx.size();
  int derivative_idx = 0;
  for (int i = 0; i < ADx.size(); ++i)
  {
    ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
    derivative_idx++;
  }

  // propagate xdot autodiff scalar at current x
  state_t<ad_state> Xdot;

  control_t<ad_state> ad_u_bar(u_bar);
  for (int i = 0; i < ad_u_bar.size(); ++i)
    ad_u_bar(i).derivatives().setZero();
  Matrix<ad_state, DroneEuler::NP, 1> ad_params(params);
  for (int i = 0; i < ad_params.size(); ++i)
    ad_params(i).derivatives().setZero();
  drone->state_dynamics(ADx, ad_u_bar, ad_params, Xdot);

  // obtain the jacobian of f(x)
  for (int i = 0; i < Xdot.size(); i++)
  {
    A.row(i) = Xdot(i).derivatives();
  }

  /** initialize derivatives */
  control_t<ad_control> ADu(u_bar);
  int div_size2 = ADu.size();
  int derivative_idx2 = 0;
  for (int i = 0; i < ADu.size(); ++i)
  {
    ADu(i).derivatives() = control::Unit(div_size2, derivative_idx2);
    derivative_idx2++;
  }

  state_t<ad_control> Xdot2;

  state_t<ad_control> ad_x_bar(x_bar);
  for (int i = 0; i < ad_x_bar.size(); ++i)
    ad_x_bar(i).derivatives().setZero();

  Matrix<ad_control, DroneEuler::NP, 1> ad_params2(params);
  for (int i = 0; i < ad_params2.size(); ++i)
    ad_params2(i).derivatives().setZero();

  drone->state_dynamics(ad_x_bar, ADu, ad_params2, Xdot2);

  // obtain the jacobian of f(x)
  for (int i = 0; i < Xdot2.size(); i++)
  {
    B.row(i) = Xdot2(i).derivatives();
  }
}

// code found in https://github.com/TakaHoribe/Riccati_Solver //TODO implement better version
bool solveRiccatiIterationC(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
                            const Eigen::MatrixXd& R, Eigen::MatrixXd& P, const double dt, const double& tolerance,
                            const uint iter_max)
{
  P = Q;  // initialize

  Eigen::MatrixXd P_next;

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd Rinv = R.inverse();

  double diff;
  for (uint i = 0; i < iter_max; ++i)
  {
    P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < tolerance)
    {
      std::cout << "iteration mumber = " << i << std::endl;
      return true;
    }
  }
  return false;  // over iteration limit
}
