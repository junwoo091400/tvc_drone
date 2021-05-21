#include "ros/ros.h"

#include <time.h>

#include <iostream>
#include <chrono>


#include "Eigen/Core"
#include "Eigen/Geometry"

#include <autodiff/AutoDiffScalar.h>
#include <drone_model.hpp>

#include "polymath.h"

static const int NX = 13;
static const int NU = 4;

using namespace Eigen;

// Autodiff dor state
template<typename scalar_t>
using state_t = Matrix<scalar_t, NX, 1>;
using state = state_t<double>;

template<typename scalar_t>
using control_t = Matrix<scalar_t, NU, 1>;
using control = control_t<double>;

using ad_state = AutoDiffScalar<state>;
using ad_control = AutoDiffScalar<control>;

void compute_linearized_model(Drone &drone,
                              Matrix<double, NX, NX> &A,
                              Matrix<double, NX, NU> &B) {
    state x_bar;
    x_bar << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    control u_bar;
    u_bar << 0.0, 0.0, drone.getHoverSpeedAverage(), 0.0;

    ROS_INFO_STREAM("u_bar" << u_bar);

    Matrix<double, 4, 1> params;
    params << 1.0, 0.0, 0.0, 0.0;


//    linearize(x_bar, u_bar, params, A, B);

    /** initialize derivatives */
    state_t<ad_state> ADx(x_bar);
    int div_size = ADx.size();
    int derivative_idx = 0;
    for (int i = 0; i < ADx.size(); ++i) {
        ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
        derivative_idx++;
    }

    //propagate xdot autodiff scalar at current x
    state_t<ad_state> Xdot;

    control_t<ad_state> ad_u_bar(u_bar);
    for (int i = 0; i < ad_u_bar.size(); ++i) ad_u_bar(i).derivatives().setZero();
    Matrix<ad_state, 4, 1> ad_params(params);
    for (int i = 0; i < ad_params.size(); ++i) ad_params(i).derivatives().setZero();
    drone.state_dynamics(ADx,
                         ad_u_bar,
                         ad_params,
                         Xdot);

    // obtain the jacobian of f(x)
    for (int i = 0; i < Xdot.size(); i++) {
        A.row(i) = Xdot(i).derivatives();
    }


    /** initialize derivatives */
    control_t<ad_control> ADu(u_bar);
    int div_size2 = ADu.size();
    int derivative_idx2 = 0;
    for (int i = 0; i < ADu.size(); ++i) {
        ADu(i).derivatives() = control::Unit(div_size2, derivative_idx2);
        derivative_idx2++;
    }

    state_t<ad_control> Xdot2;

    state_t<ad_control> ad_x_bar(x_bar);
    for (int i = 0; i < ad_x_bar.size(); ++i) ad_x_bar(i).derivatives().setZero();

    Matrix<ad_control, 4, 1> ad_params2(params);
    for (int i = 0; i < ad_params2.size(); ++i) ad_params2(i).derivatives().setZero();

    drone.state_dynamics(ad_x_bar,
                         ADu,
                         ad_params2,
                         Xdot2);

    // obtain the jacobian of f(x)
    for (int i = 0; i < Xdot2.size(); i++) {
        B.row(i) = Xdot2(i).derivatives();
    }
}

void compute_LQR_terminal_cost(Drone &drone, Matrix<double, NX - 1, NX - 1> &QN) {
    Matrix<double, NX, NX> A_;
    Matrix<double, NX, NU> B_;


    compute_linearized_model(drone, A_, B_);

    Matrix<double, NX - 1, NX - 1> A;
    A.block(0, 0, 9, 9) = A_.block(0, 0, 9, 9);
    A.block(9, 0, 3, 9) = A_.block(10, 0, 3, 9);
    A.block(0, 9, 9, 3) = A_.block(0, 10, 9, 3);
    A.block(9, 9, 3, 3) = A_.block(10, 10, 3, 3);
    Matrix<double, NX - 1, NU> B;
    B.block(0, 0, 9, 4) = B_.block(0, 0, 9, 4);
    B.block(9, 0, 3, 4) = B_.block(10, 0, 3, 4);

    ROS_INFO_STREAM("A = DF/DX\n" << A);
    ROS_INFO_STREAM("B = DF/DU\n" << B);
    Matrix<double, NX - 1, NX - 1> C;
    C.setOnes();
//    polymath::LinearSystem sys(A,B,C);

    Matrix<double, NX - 1, NX - 1> Q;
    Q.setZero();
    Q.diagonal() << 1, 1, 5,
            0.1, 0.1, 0.5,
            1, 1, 1,
            1, 1, 1;
    Matrix<double, NU, NU> R;
    R.setZero();
    R.diagonal() << 5, 5, 0.01, 0.01;

    QN = polymath::oc::care(A, B * R.inverse() * B.transpose(), Q);
//    Eigen::MatrixXd K = polymath::oc::lqr(sys, Q, R, M);

}