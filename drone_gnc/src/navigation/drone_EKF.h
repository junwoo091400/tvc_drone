//
// Created by raphlins on 10.04.21.
//

#ifndef SRC_DRONE_EKF_H
#define SRC_DRONE_EKF_H

#include "ros/ros.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <autodiff/AutoDiffScalar.h>

using namespace Eigen;

class DroneEKF {
public:
    static const int NX = 13;
    static const int NU = 4;
    // Autodiff dor state
    template<typename scalar_t>
    using state_t = Matrix<scalar_t, NX, 1>;

    using ADScalar = AutoDiffScalar<state_t<double>>;
    using ad_state_t = state_t<ADScalar>;

    // Autodiff for sensor
    template<typename scalar_t>
    using sensor_t = Matrix<scalar_t, 6, 1>;
    template<typename scalar_t>
    using optitrack_sensor_t = Matrix<scalar_t, 7, 1>;

    typedef Matrix<double, NX, NX> state_matrix;

    double last_predict_time;
    state_t<double> X;

    template<typename T>
    void state_dynamics(const state_t<T> &x, state_t<T> &xdot);

    void init_EKF(ros::NodeHandle &nh);

    void RK4(const state_t<double> &X, double dT, state_t<double> &Xnext);

    void P_derivative(const state_matrix &P, state_matrix &Pdot);

    void RK4_P(const state_matrix &P, double dT, state_matrix &Pnext);

    void predict_step();

    void update_step(sensor_t<double> z);

    void optitrack_update_step(optitrack_sensor_t<double> z);

private:
    state_matrix Q;
    Matrix<double, 6, 6> R;
    Matrix<double, 7, 7> R_optitrack;

    state_matrix P;

    state_matrix F;

    Matrix<double, 6, NX> H;
    Matrix<double, 7, NX> H_optitrack;


};


#endif //SRC_DRONE_EKF_H
