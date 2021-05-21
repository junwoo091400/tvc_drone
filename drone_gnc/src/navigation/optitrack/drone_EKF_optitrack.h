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
    static const int NX = 17;
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

    typedef Matrix<double, NX, NX> state_matrix;

    double last_predict_time;
    state X;

    ad_state ADx;

    template<typename T>
    void stateDynamics(const state_t<T> &x, state_t<T> &xdot);

    template<typename T>
    void measurementModel(const state_t<T> &x, sensor_data_t<T> &z);

    DroneEKF(ros::NodeHandle &nh);

    void fullDerivative(const state &x, const state_matrix &P, state &xdot, state_matrix &Pnext);

    void RK4(const state &X, const state_matrix &P, double dT, state &Xnext, state_matrix &Pnext);

    void predictStep();

    void updateStep(sensor_data_t<double> z);

    void updateCurrentControl(const drone_gnc::DroneControl::ConstPtr &drone_control);

private:
    state_matrix Q;
    Matrix<double, NZ, NZ> R;

    state_matrix P;

    state_matrix F;
    Matrix<double, NZ, NX> H;

    Drone drone;
    drone_gnc::DroneControl current_control;
    bool received_control;
};


#endif //SRC_DRONE_EKF_OPTITRACK_H
