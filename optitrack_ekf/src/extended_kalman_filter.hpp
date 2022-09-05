/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "ros/ros.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "AutoDiffScalar.h"

using namespace Eigen;

class ExtendedKalmanFilter
{
public:
  static const int NX = 13;
  static const int NZ = 7;

  // template variables for Autodiff to compute F and H
  // Autodiff for state
  template <typename scalar_t>
  using state_t = Matrix<scalar_t, NX, 1>;
  using state = state_t<double>;
  // Autodiff state variable
  using ad_state = state_t<AutoDiffScalar<state>>;

  // Autodiff for sensor
  template <typename scalar_t>
  using sensor_data_t = Matrix<scalar_t, NZ, 1>;
  using sensor_data = sensor_data_t<double>;
  // Autodiff sensor variable
  using ad_sensor_data = sensor_data_t<AutoDiffScalar<state_t<double>>>;

  typedef Matrix<double, NX, NX> state_matrix;
  typedef Matrix<double, NZ, NZ> sensor_matrix;

private:
  // EKF usual matrices
  state_matrix Q;            // constant
  sensor_matrix R;           // constant
  state_matrix F;            // computed using autodiff
  Matrix<double, NZ, NX> H;  // computed using autodiff

  state X0;
  state X;
  state_matrix P;
  ad_state ADx;

public:
  ExtendedKalmanFilter()
  {
    Q.setIdentity();
    R.setIdentity();
    X.setZero();
    H.setZero();

    // initialize derivatives ->  the Jacobian of ADx (=X but in autodiff format) is the identity matrix
    ADx << X;
    int div_size = ADx.size();
    int derivative_idx = 0;
    for (int i = 0; i < ADx.size(); ++i)
    {
      ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
      derivative_idx++;
    }
  }

  void init(state_matrix& Q_, sensor_matrix& R_, state& X0)
  {
    Q = Q_;
    R = R_;

    P = Q;
    X = X0;
  }

  state getState()
  {
    return X;
  }

  template <typename T>
  void stateDynamics(const state_t<T>& x, state_t<T>& xdot)
  {
    // use 3d rigid body dynamics
    Eigen::Quaternion<T> attitude(x(6), x(3), x(4), x(5));
    attitude.normalize();
    // Angular velocity omega in quaternion format to compute quaternion derivative
    Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

    // state derivatives
    xdot.segment(0, 3) = x.segment(7, 3);
    xdot.segment(3, 4) = 0.5 * (attitude * omega_quat).coeffs();
    xdot.segment(7, 3) << 0.0, 0.0, 0.0;
    xdot.segment(10, 3) << 0.0, 0.0, 0.0;
  }

  template <typename T>
  void measurementModel(const state_t<T>& x, sensor_data_t<T>& z)
  {
    z.segment(0, 3) = x.segment(0, 3);
    z.segment(3, 4) = x.segment(3, 4);
  }

  void fullDerivative(const state& x, const state_matrix& P, state& xdot, state_matrix& Pdot)
  {
    // X derivative
    stateDynamics(x, xdot);

    // P derivative
    // propagate xdot autodiff scalar at current x
    ADx = X;
    ad_state Xdot;
    stateDynamics(ADx, Xdot);

    // fetch the jacobian of f(x)
    for (int i = 0; i < Xdot.size(); i++)
    {
      F.row(i) = Xdot(i).derivatives();
    }

    Pdot = F * P + P * F.transpose() + Q;
  }

  void RK4(const state& X, const state_matrix& P, double dT, state& Xnext, state_matrix& Pnext)
  {
    state k1, k2, k3, k4;
    state_matrix k1_P, k2_P, k3_P, k4_P;

    fullDerivative(X, P, k1, k1_P);
    fullDerivative(X + k1 * dT / 2, P + k1_P * dT / 2, k2, k2_P);
    fullDerivative(X + k2 * dT / 2, P + k2_P * dT / 2, k3, k3_P);
    fullDerivative(X + k3 * dT, P + k3_P * dT, k4, k4_P);

    Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    Pnext = P + (k1_P + 2 * k2_P + 2 * k3_P + k4_P) * dT / 6;
  }

  void predictStep(double dT)
  {
    // predict: integrate X and P
    RK4(X, P, dT, X, P);
  }

  void updateStep(sensor_data& z)
  {
    // propagate hdot autodiff scalar at current x
    ADx = X;
    ad_sensor_data hdot;
    measurementModel(ADx, hdot);

    // compute h(x)
    sensor_data h_x;
    measurementModel(X, h_x);

    // obtain the jacobian of h(x)
    for (int i = 0; i < hdot.size(); i++)
    {
      H.row(i) = hdot(i).derivatives();
    }

    // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
    // this is a faster version of the commented code below
    Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
    Eigen::Matrix<double, NZ, NZ> S;    // innovation covariance
    Eigen::Matrix<double, NX, NZ> K;    // Kalman gain
    Eigen::Matrix<double, NX, NX> I;    // identity
    I.setIdentity();
    S = H * P * H.transpose() + R;
    K = S.llt().solve(H * P).transpose();
    X = X + K * (z - h_x);
    IKH = (I - K * H);
    P = IKH * P * IKH.transpose() + K * R * K.transpose();

    // equivalent code:
    // Matrix<double, NX, NZ> K;
    // K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    // X = X + K * (z - h_x);
    // P = P - K * H * P;
  }
};

#endif
