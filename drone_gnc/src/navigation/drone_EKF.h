#ifndef SRC_DRONE_EKF_H
#define SRC_DRONE_EKF_H

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
    // Autodiff dor state
    template<typename scalar_t>
    using state_t = Matrix<scalar_t, NX, 1>;
    using state = state_t<double>;

    using ADScalar = AutoDiffScalar<state_t<double>>;
    using ad_state_t = state_t<ADScalar>;

    // Autodiff for sensor
    template<typename scalar_t>
    using sensor_t = Matrix<scalar_t, 6, 1>;
    template<typename scalar_t>
    using optitrack_sensor_t = Matrix<scalar_t, 7, 1>;

    typedef Matrix<double, NX, NX> state_matrix;
//    typedef Matrix<double, NX, 1> state;

    double last_predict_time;
    state X;

    ad_state_t ADx;

    template<typename T>
    void stateDynamics(const state_t<T> &x, state_t<T> &xdot);

    void initEKF(ros::NodeHandle &nh);

    void fullDerivative(const state &x, const state_matrix &P, state &xdot, state_matrix &Pnext);

    void RK4(const state &X, const state_matrix &P, double dT, state &Xnext, state_matrix &Pnext);

    void predictStep();

    void updateStep(sensor_t<double> z);

    void optitrackUpdateStep(optitrack_sensor_t<double> z);

    void updateCurrentControl(const drone_gnc::DroneControl::ConstPtr &drone_control);

private:
    state_matrix Q;
    Matrix<double, 6, 6> R;
    Matrix<double, 7, 7> R_optitrack;

    state_matrix P;

    state_matrix F;

    Matrix<double, 6, NX> H;
    Matrix<double, 7, NX> H_optitrack;

    Drone drone;
    drone_gnc::DroneControl current_control;
    bool received_control;
};


#endif //SRC_DRONE_EKF_H
