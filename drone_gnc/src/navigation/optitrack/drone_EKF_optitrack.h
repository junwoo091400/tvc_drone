#ifndef SRC_DRONE_EKF_OPTITRACK_H
#define SRC_DRONE_EKF_OPTITRACK_H

#include "ros/ros.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <autodiff/AutoDiffScalar.h>
#include <drone_model.hpp>
#include <drone_gnc/DroneControl.h>

using namespace Eigen;

class DroneEKF {
public:
    static const int NX = 23;
    static const int NU = 4;
    static const int NZ = 7;

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

    using control = Matrix<double, 4, 1>;

    typedef Matrix<double, NX, NX> state_matrix;

    state X;
    ad_state ADx;
    bool estimate_params;
    bool received_control;

    DroneEKF(ros::NodeHandle &nh);
    void reset();

    void setQdiagonal(const state &Qdiag);

    void setRdiagonal(const sensor_data &Rdiag);

    template<typename T>
    void stateDynamics(const state_t<T> &x, const control &u, state_t<T> &xdot);

    template<typename T>
    void measurementModel(const state_t<T> &x, sensor_data_t<T> &z);

    void fullDerivative(const state &x, const state_matrix &P, const control &u, state &xdot, state_matrix &Pnext);

    void RK4(const state &X, const state_matrix &P, const control &u, double dT, state &Xnext, state_matrix &Pnext);

    void predictStep(double dT, const control &u);

    void updateStep(sensor_data_t<double> &z);

    void updateCurrentControl(const drone_gnc::DroneControl::ConstPtr &drone_control);

private:
    state_matrix Q;
    Matrix<double, NZ, NZ> R;

    state_matrix P;

    state_matrix F;
    Matrix<double, NZ, NX> H;

    Drone drone;

    state X0;
};


#endif //SRC_DRONE_EKF_OPTITRACK_H
