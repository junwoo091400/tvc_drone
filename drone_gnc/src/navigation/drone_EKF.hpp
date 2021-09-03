#ifndef SRC_DRONE_EKF_HPP
#define SRC_DRONE_EKF_HPP

#include "ros/ros.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <autodiff/AutoDiffScalar.h>
#include <drone_model.hpp>
#include <drone_gnc/DroneControl.h>

using namespace Eigen;

template <typename Derived, int NZ>
class DroneEKF {
public:
    static const int NP = Drone::NP;
    static const int NU = Drone::NU;
    static const int NX = Drone::NX + Drone::NP;

    // Autodiff dor state
    template<typename scalar_t>
    using state_t = Matrix<scalar_t, NX, 1>;
    using state = state_t<double>;

    using ad_state = state_t<AutoDiffScalar<state_t<double>>>;

    // Autodiff for sensor
    template<typename scalar_t>
    using sensor_data_t = Matrix<scalar_t, NZ, 1>;
    using sensor_data = sensor_data_t<double>;

    using ad_sensor_data = sensor_data_t<AutoDiffScalar<state_t<double>>>;

    typedef Matrix<double, NX, NX> state_matrix;

    state X;
    ad_state ADx;
    bool estimate_params;
    bool received_control;

    DroneEKF(ros::NodeHandle &nh) : drone(nh) {
        std::vector<double> initial_state;
        nh.getParam("initial_state", initial_state);
        Drone::state drone_X0(initial_state.data());

        bool init_estimated_params;
        nh.param("init_estimated_params", init_estimated_params, false);

        if (init_estimated_params) {
            double thrust_scaling, torque_scaling, servo1_offset, servo2_offset;
            nh.param<double>("/rocket/estimated/thrust_scaling", thrust_scaling, 1);
            nh.param<double>("/rocket/estimated/torque_scaling", torque_scaling, 1);
            nh.param<double>("/rocket/estimated/servo1_offset", servo1_offset, 0);
            nh.param<double>("/rocket/estimated/servo2_offset", servo2_offset, 0);

            X0 << drone_X0, thrust_scaling, torque_scaling, servo1_offset, servo2_offset, 0, 0, 0, 0, 0, 0;
        } else {
            X0 << drone_X0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0;
        }

        nh.param("estimate_params", estimate_params, false);

        Q.setIdentity();
        R.setIdentity();

        reset();

        /** initialize derivatives */
        ADx(X);
        int div_size = ADx.size();
        int derivative_idx = 0;
        for (int i = 0; i < ADx.size(); ++i) {
            ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
            derivative_idx++;
        }
    }

    void reset() {
        X = X0;
        P = Q;
        H.setZero();

        received_control = false;
    }

    void startParamEstimation(){
        P = Q;
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
        if (received_control && estimate_params) {
            Drone::state_t<T> x_drone = x.head(Drone::NX);

            Drone::parameters_t<T> params = x.segment(Drone::NX, Drone::NP);

            Drone::control_t<T> u_drone = u;

            //state derivatives
            drone.state_dynamics(x_drone, u_drone, params, xdot);
        } else {
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
        xdot.segment(13, 10).setZero();
    }

    template<typename T>
    void measurementModel(const state_t<T> &x, sensor_data_t<T> &z){
        static_cast<Derived*>(this)->measurementModel_impl(x, z);
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

        // obtain the jacobian of f(x)
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

        // copied from https://github.com/LA-EPFL/yakf/blob/master/ExtendedKalmanFilter.h
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

//    Matrix<double, NX, NZ> K;
//    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
//    X = X + K * (z - h_x);
//    P = P - K * H * P;
    }
private:
    state_matrix P;

    state_matrix F;
    Matrix<double, NZ, NX> H;

    Drone drone;

    state X0;
protected:
    state_matrix Q;
    Matrix<double, NZ, NZ> R;
};

//template <typename Derived> const int DroneEKF<Derived>::NZ(Derived::NZ);


#endif //SRC_DRONE_EKF_HPP
