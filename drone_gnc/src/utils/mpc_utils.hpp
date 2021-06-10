#include "ros/ros.h"

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
#include <drone_model.hpp>

/** Lyapunov equation */
Eigen::MatrixXd
lyapunov(const Eigen::Ref<const Eigen::MatrixXd> A, const Eigen::Ref<const Eigen::MatrixXd> Q) noexcept {
    const Eigen::Index m = Q.cols();
    /** compute Schur decomposition of A */
    Eigen::RealSchur <Eigen::MatrixXd> schur(A);
    Eigen::MatrixXd T = schur.matrixT();
    Eigen::MatrixXd U = schur.matrixU();

    Eigen::MatrixXd Q1;
    Q1.noalias() = (U.transpose() * Q) * U;
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(m, m);
    Eigen::MatrixXd E = Eigen::MatrixXd::Identity(m, m);

    X.col(m - 1).noalias() = (T + T(m - 1, m - 1) * E).inverse();
    Eigen::VectorXd v;

    for (Eigen::Index i = m - 2; i >= 0; --i) {
        v.noalias() = Q1.col(i) - X.block(0, i + 1, m, m - (i + 1)) * T.block(i, i + 1, 1, m - (i + 1)).transpose();
        X.col(i) = (T + T(i, i) * E).partialPivLu().solve(v);
    }

    X.noalias() = (U * X) * U.transpose();
    return X;
}

double line_search_care(const double &a, const double &b, const double &c) noexcept {
    Eigen::Matrix<double, 5, 1> poly;
    Eigen::Matrix<double, 4, 1> poly_derivative;
    poly_derivative << -2 * a, 2 * (a - 2 * b), 6 * b, 4 * c;
    poly << a, -2 * a, a - 2 * b, 2 * b, c;
    poly_derivative = (1.0 / (4 * c)) * poly_derivative;
    poly = (1.0 / c) * poly;

    /** find extremums */
    Eigen::PolynomialSolver<double, 3> root_finder;
    root_finder.compute(poly_derivative);

    /** compute values on the bounds */
    double lb_value = Eigen::poly_eval(poly, 1e-5);
    double ub_value = Eigen::poly_eval(poly, 2);

    double argmin = lb_value < ub_value ? 1e-5 : 2;

    /** check critical points : redo with visitor! */
    double minimum = Eigen::poly_eval(poly, argmin);
    for (int i = 0; i < root_finder.roots().size(); ++i) {
        double root = root_finder.roots()(i).real();
        if ((root >= 1e-5) && (root <= 2)) {
            double candidate = Eigen::poly_eval(poly, root);
            if (candidate < minimum) {
                argmin = root;
                minimum = Eigen::poly_eval(poly, argmin);
            }
        }
    }
    return argmin;
}

/** CARE Newton iteration */
Eigen::MatrixXd newton_ls_care(const Eigen::Ref<const Eigen::MatrixXd> A, const Eigen::Ref<const Eigen::MatrixXd> B,
                               const Eigen::Ref<const Eigen::MatrixXd> C,
                               const Eigen::Ref<const Eigen::MatrixXd> X0) noexcept {
    /** initial guess */
    //Eigen::EigenSolver<Eigen::MatrixXd> eig(A - B * X0);
    //std::cout << "INIT X0: \n" << eig.eigenvalues() << "\n";
    const double tol = 1e-5;
    const int kmax = 20;
    Eigen::MatrixXd X = X0;
    double err = std::numeric_limits<double>::max();
    int k = 0;
    Eigen::MatrixXd RX, H, V;

    /** temporary */
    double tk = 1;

    while ((err > tol) && (k < kmax)) {
        RX = C;
        RX.noalias() += X * A + A.transpose() * X - (X * B) * X;
        /** newton update */
        H = lyapunov((A - B * X).transpose(), -RX);
        /** exact line search */
        V.noalias() = H * B * H;
        double a = (RX * RX).trace();
        double b = (RX * V).trace();
        double c = (V * V).trace();
        tk = line_search_care(a, b, c);
        /** inner loop to accept step */
        X.noalias() += tk * H;
        //err = tk * (H.lpNorm<1>() / X.lpNorm<1>());
        err = RX.norm();
        //std::cout << "err " << err << " step " << tk << "\n";
        k++;
    }

    /** may be defect correction algorithm? */

    //std::cout << "CARE solve took " << k << " iterations. \n";
    if (k == kmax)
        std::cerr << "CARE cannot be solved to specified precision :" << err
                  << " max number of iteration exceeded! \n ";

    return X;
}

/** Moore-Penrose pseudo-inverse */
Eigen::MatrixXd pinv(const Eigen::Ref<const Eigen::MatrixXd> mat) noexcept {
    /** compute SVD */
    Eigen::JacobiSVD <Eigen::MatrixXd> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const double pinvtol = 1e-6;
    Eigen::VectorXd singular_values = svd.singularValues();
    /** make a copy */
    Eigen::VectorXd singular_values_inv = singular_values;
    for (int i = 0; i < mat.cols(); ++i) {
        if (singular_values(i) > pinvtol)
            singular_values_inv(i) = 1.0 / singular_values(i);
        else singular_values_inv(i) = 0;
    }
    return (svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose());
}

Eigen::MatrixXd
init_newton_care(const Eigen::Ref<const Eigen::MatrixXd> A, const Eigen::Ref<const Eigen::MatrixXd> B) noexcept {
    const Eigen::Index n = A.cols();
    const double tolerance = 1e-12;
    /** compute Schur decomposition of A */
    Eigen::RealSchur <Eigen::MatrixXd> schur(A);
    Eigen::MatrixXd TA = schur.matrixT();
    Eigen::MatrixXd U = schur.matrixU();

    Eigen::MatrixXd TD = U.transpose() * B;
    Eigen::EigenSolver <Eigen::MatrixXd> es;
    es.compute(TA, false);

    Eigen::VectorXd eig_r = es.eigenvalues().real();
    double b = -eig_r.minCoeff();
    b = std::fmax(b, 0.0) + 0.5;
    Eigen::MatrixXd E = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd Z = lyapunov(TA + b * E, 2 * TD * TD.transpose());
    Eigen::MatrixXd X = (TD.transpose() * pinv(Z)) * U.transpose();

    if ((X - X.transpose()).norm() > tolerance) {
        Eigen::MatrixXd M = (X.transpose() * B) * X + 0.5 * Eigen::MatrixXd::Identity(n, n);
        X = lyapunov((A - B * X).transpose(), -M);
    }
    return X;
}


Eigen::MatrixXd care(const Eigen::Ref<const Eigen::MatrixXd> A, const Eigen::Ref<const Eigen::MatrixXd> B,
                     const Eigen::Ref<const Eigen::MatrixXd> C) noexcept {
    Eigen::MatrixXd X0 = init_newton_care(A, B);
    return newton_ls_care(A, B, C, X0);
}

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

    Drone::parameters params;
    params << 1.0, 1.0,
            0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;


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
    Matrix<ad_state, Drone::NP, 1> ad_params(params);
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

    Matrix<ad_control, Drone::NP, 1> ad_params2(params);
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

    Matrix < double, NX - 1, NX - 1 > A;
    A.block(0, 0, 9, 9) = A_.block(0, 0, 9, 9);
    A.block(9, 0, 3, 9) = A_.block(10, 0, 3, 9);
    A.block(0, 9, 9, 3) = A_.block(0, 10, 9, 3);
    A.block(9, 9, 3, 3) = A_.block(10, 10, 3, 3);
    Matrix < double, NX - 1, NU > B;
    B.block(0, 0, 9, 4) = B_.block(0, 0, 9, 4);
    B.block(9, 0, 3, 4) = B_.block(10, 0, 3, 4);

    Matrix < double, NX - 1, NX - 1 > Q;
    Q.setZero();
    Q.diagonal() <<
                 2, 2, 4,
            2, 2, 6,
            2, 2, 1e-10,
            2, 2, 5;
    Matrix<double, NU, NU> R;
    R.setZero();
    R.diagonal() << 5, 5, 0.01, 0.01;

    ROS_INFO_STREAM("A = DF/DX\n" << A);
    ROS_INFO_STREAM("B = DF/DU\n" << B);
    ROS_INFO_STREAM("Q\n" << Q);
    ROS_INFO_STREAM("R\n" << R);

    QN = care(A, B * R.inverse() * B.transpose(), Q);
    ROS_INFO_STREAM("QN\n" << QN);
//    Matrix < double, NX - 1, NX - 1 > C;
//    C.setOnes();
//    polymath::LinearSystem sys(A,B,C);
//    Eigen::MatrixXd K = polymath::oc::lqr(sys, Q, R, M);

}

