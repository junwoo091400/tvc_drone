/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_DRONE_EKF_HPP
#define SRC_DRONE_EKF_HPP

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <autodiff/AutoDiffScalar.h>
#include <drone_model.hpp>
#include "navigation_settings.hpp"

using namespace Eigen;

//template<typename Derived, int NZ>
class DroneEKF {
public:
    static const int NP = Drone::NP;
    static const int NU = Drone::NU;
    static const int NX = Drone::NX + Drone::NP;
    static const int NZ = 13;

    // template variables for Autodiff to compute F and H
    // Autodiff for state
    template<typename scalar_t>
    using state_t = Matrix<scalar_t, NX, 1>;
    using state = state_t<double>;
    // Autodiff state variable
    using ad_state = state_t<AutoDiffScalar<state>>;

    // Autodiff for sensor
    template<typename scalar_t>
    using sensor_data_t = Matrix<scalar_t, NZ, 1>;
    using sensor_data = sensor_data_t<double>;
    // Autodiff sensor variable
    using ad_sensor_data = sensor_data_t<AutoDiffScalar<state_t<double>>>;

    typedef Matrix<double, NX, NX> state_matrix;
    typedef Matrix<double, NZ, NZ> sensor_matrix;

private:
    // EKF usual matrices
    state_matrix Q; // constant
    sensor_matrix R; // constant
    state_matrix F; // computed using autodiff
    Matrix<double, NZ, NX> H; // computed using autodiff

    state X0;
    state X;
    state_matrix P;
    ad_state ADx;

    DroneProps<double> drone_props;
    Drone *drone;

    NavigationSettings<double> settings;

public:
    bool received_control = false;

    DroneEKF(Drone *drone, NavigationSettings<double> settings) : drone(drone), settings(settings) {
        Drone::state drone_X0(settings.initial_state.data());

        if (settings.init_estimated_params) {
            X0 << drone_X0, settings.initial_thrust_scaling, settings.initial_torque_scaling, 0, 0, 0, 0, 0, 0;
        } else {
            X0 << drone_X0, 1, 1, 0, 0, 0, 0, 0, 0;
        }

        Q.setIdentity();
        R.setIdentity();

        reset();

        // initialize derivatives ->  the Jacobian of ADx (=X but in autodiff format) is the identity matrix
        ADx = X;
        int div_size = ADx.size();
        int derivative_idx = 0;
        for (int i = 0; i < ADx.size(); ++i) {
            ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
            derivative_idx++;
        }

        state diag_Q;
        diag_Q << settings.x_var, settings.x_var, settings.x_var,
                settings.dx_var, settings.dx_var, settings.dx_var,
                settings.att_var, settings.att_var, settings.att_var, settings.att_var,
                settings.datt_var, settings.datt_var, settings.datt_var,
                settings.thrust_scaling_var,
                settings.torque_scaling_var,
                settings.disturbance_force_var, settings.disturbance_force_var, settings.disturbance_force_z_var,
                settings.disturbance_torque_var, settings.disturbance_torque_var, settings.disturbance_torque_z_var;
        setQdiagonal(diag_Q);

        sensor_data diag_R;
        if (settings.use_gps) {
            diag_R << settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var,
                    settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var,
                    settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var,
                    settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var;
        } else {
            diag_R << settings.x_optitrack_var, settings.x_optitrack_var, settings.x_optitrack_var,
                    settings.pixhawk_vel_var, settings.pixhawk_vel_var, settings.pixhawk_vel_var,
                    settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var,
                    settings.pixhawk_var, settings.pixhawk_var, settings.pixhawk_var;
        }
        setRdiagonal(diag_R);

    }

    state getState() {
        return X;
    }

    double getState(int i) {
        return X(i);
    }

    void reset() {
        X = X0;
        P = Q;
        H.setZero();

        received_control = false;
    }

    void startParamEstimation() {
        state Pdiag = P.diagonal();
        P.setIdentity();
        P.diagonal().head(Drone::NX) << Pdiag.head(Drone::NX);
        received_control = true;
    }

    void setQdiagonal(const state &Qdiag) {
        Q.diagonal() << Qdiag;
    }

    void setRdiagonal(const sensor_data &Rdiag) {
        R.diagonal() << Rdiag;
    }

    template<typename T>
    void stateDynamics(const state_t<T> &x, const Drone::control &u, state_t<T> &xdot) {
        if (received_control && settings.estimate_params) {
            // use full dynamics to estimate parameters
            Drone::state_t<T> x_drone = x.head(Drone::NX);

            Drone::parameters_t<T> params = x.segment(Drone::NX, Drone::NP);

            Drone::control_t<T> u_drone = u;

            //state derivatives
            drone->state_dynamics(x_drone, u_drone, params, xdot);
        } else {
            // use 3d rigid body dynamics without parameter estimation
            // Orientation of the rocket with quaternion
            Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
            attitude.normalize();
            // Angular velocity omega in quaternion format to compute quaternion derivative
            Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

            //state derivatives
            xdot.segment(0, 3) = x.segment(3, 3);
            xdot.segment(3, 3) << 0.0, 0.0, 0.0;
            xdot.segment(6, 4) = 0.5 * (attitude * omega_quat).coeffs();
            xdot.segment(10, 3) << 0.0, 0.0, 0.0;
        }

        // assume parameters unchanged
        xdot.segment(Drone::NX, Drone::NP).setZero();
    }

    template<typename T>
    void measurementModel(const state_t<T> &x, sensor_data_t<T> &z) {
        //static_cast<Derived *>(this)->measurementModel_impl(x, z);
        z.head(13) = x.head(13);
    }

    void fullDerivative(const state &x,
                        const state_matrix &P,
                        const Drone::control &u,
                        state &xdot,
                        state_matrix &Pdot) {
        //X derivative
        stateDynamics(x, u, xdot);

        //P derivative
        //propagate xdot autodiff scalar at current x
        ADx = X;
        ad_state Xdot;
        stateDynamics(ADx, u, Xdot);

        // fetch the jacobian of f(x)
        for (int i = 0; i < Xdot.size(); i++) {
            F.row(i) = Xdot(i).derivatives();
        }

        Pdot = F * P + P * F.transpose() + Q;
    }


    void RK4(const state &X,
             const state_matrix &P,
             const Drone::control &u,
             double dT,
             state &Xnext,
             state_matrix &Pnext) {
        state k1, k2, k3, k4;
        state_matrix k1_P, k2_P, k3_P, k4_P;

        fullDerivative(X, P, u, k1, k1_P);
        fullDerivative(X + k1 * dT / 2, P + k1_P * dT / 2, u, k2, k2_P);
        fullDerivative(X + k2 * dT / 2, P + k2_P * dT / 2, u, k3, k3_P);
        fullDerivative(X + k3 * dT, P + k3_P * dT, u, k4, k4_P);

        Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
        Pnext = P + (k1_P + 2 * k2_P + 2 * k3_P + k4_P) * dT / 6;
    }


    void predictStep(double dT, const Drone::control &u) {
        //predict: integrate X and P
        RK4(X, P, u, dT, X, P);
    }

    void updateStep(sensor_data &z) {
        //propagate hdot autodiff scalar at current x
        ADx = X;
        ad_sensor_data hdot;
        measurementModel(ADx, hdot);

        //compute h(x)
        sensor_data h_x;
        measurementModel(X, h_x);

        // obtain the jacobian of h(x)
        for (int i = 0; i < hdot.size(); i++) {
            H.row(i) = hdot(i).derivatives();
        }

        // taken from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
        // this is a faster version of the commented code below
        Eigen::Matrix<double, NX, NX> IKH;  // temporary matrix
        Eigen::Matrix<double, NZ, NZ> S; // innovation covariance
        Eigen::Matrix<double, NX, NZ> K; // Kalman gain
        Eigen::Matrix<double, NX, NX> I; // identity
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

//template <typename Derived> const int DroneEKF<Derived>::NZ(Derived::NZ);


#endif //SRC_DRONE_EKF_HPP
