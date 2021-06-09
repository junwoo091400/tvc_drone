
#ifndef SRC_DRONE_GUIDANCE_OCP_HPP
#define SRC_DRONE_GUIDANCE_OCP_HPP

#include "ros/ros.h"

#include <string>
#include <math.h>

#include <chrono>

#include "../../../submodule/polympc/src/polynomials/ebyshev.hpp"
#include "../../../submodule/polympc/src/control/continuous_ocp.hpp"
#include "../../../submodule/polympc/src/polynomials/splines.hpp"

#include "../../../submodule/polympc/src/solvers/sqp_base.hpp"
#include "../../../submodule/polympc/src/solvers/osqp_interface.hpp"
#include "../../../submodule/polympc/src/control/mpc_wrapper.hpp"
#include "drone_model.hpp"

using namespace Eigen;

// Poly MPC stuff ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

using namespace std;

#define POLY_ORDER 7
#define NUM_SEG    2

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ guidance_ocp, /*NX*/ 13, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/2, /*TYPE*/ double)

class guidance_ocp : public ContinuousOCP<guidance_ocp, Approximation, SPARSE> {
public:
    ~guidance_ocp() = default;

    Matrix<scalar_t, 11, 1> Q;
    Matrix<scalar_t, Drone::NU, 1> R;
    Matrix<scalar_t, 11, 11> QN;

    Matrix<scalar_t, Drone::NX, 1> xs;
    Matrix<scalar_t, Drone::NU, 1> us;

    shared_ptr<Drone> drone;

    scalar_t maxAttitudeAngle;

    scalar_t min_z;
    scalar_t min_dz;
    scalar_t max_dx;
    scalar_t max_dz;
    scalar_t max_datt;
    scalar_t scaling_x;
    scalar_t scaling_z;

    Matrix<scalar_t, NX, 1> x_unscaling_vec;
    Matrix<scalar_t, NU, 1> u_unscaling_vec;

    Matrix<scalar_t, NX, 1> x_scaling_vec;
    Matrix<scalar_t, NU, 1> u_scaling_vec;

    Matrix<scalar_t, Drone::NX, 1> x_drone_unscaling_vec;
    Matrix<scalar_t, Drone::NU, 1> u_drone_unscaling_vec;
    Matrix<scalar_t, Drone::NX, 1> x_drone_scaling_vec;
    Matrix<scalar_t, Drone::NU, 1> u_drone_scaling_vec;

    void init(ros::NodeHandle nh, shared_ptr<Drone> drone_ptr) {
        drone = drone_ptr;
//        scalar_t x_cost, dx_cost, z_cost, dz_cost, att_cost, datt_cost, servo_cost, thrust_cost, torque_cost, droll_cost;
        scalar_t maxAttitudeAngle_degree, weight_scaling;
        if (nh.getParam("mpc/min_z", min_z) &&
            nh.getParam("mpc/min_dz", min_dz) &&
            nh.getParam("mpc/max_dx", max_dx) &&
            nh.getParam("mpc/max_dz", max_dz) &&
            nh.getParam("mpc/max_datt", max_datt) &&
            nh.getParam("mpc/scaling_x", scaling_x) &&
            nh.getParam("mpc/scaling_z", scaling_z) &&
            nh.getParam("mpc/weight_scaling", weight_scaling) &&

            //                nh.getParam("/mpc/state_costs/x", x_cost) &&
            //            nh.getParam("/mpc/state_costs/dz", dx_cost) &&
            //            nh.getParam("/mpc/state_costs/z", z_cost) &&
            //            nh.getParam("/mpc/state_costs/dz", dz_cost) &&
            //            nh.getParam("/mpc/state_costs/att", att_cost) &&
            //            nh.getParam("/mpc/state_costs/datt", datt_cost) &&
            //            nh.getParam("/mpc/state_costs/droll", droll_cost) &&
            //            nh.getParam("/mpc/input_costs/servo", servo_cost) &&
            //            nh.getParam("/mpc/input_costs/thrust", thrust_cost) &&
            //            nh.getParam("/mpc/input_costs/torque", torque_cost) &&

            nh.getParam("mpc/max_attitude_angle", maxAttitudeAngle_degree)) {

//            Q << x_cost, x_cost, z_cost,
//                    dx_cost, dx_cost, dz_cost,
//                    0, 0, 0, 0,
//                    datt_cost, datt_cost, droll_cost;
            Q << 4, 4, 6,
                    0.1, 0.1, 0.5,
                    2, 2,
                    1, 1, 5;
            R << 5, 5, 0.01, 0.01;

            QN << 3.6957, 0, 0, 1.6573, 0, 0, 0, 8.2092, 0, 0.44984, 0
                    , 0, 3.677, 0, 0, 1.64, 0, -8.0246, 0, -0.41549, 0, 0
                    , 0, 0, 4.0847, 0, 0, 1.1404, 0, 0, 0, 0, 0
                    , 1.6573, 0, 0, 1.1128, 0, 0, 0, 6.6851, 0, 0.32655, 0
                    , 0, 1.64, 0, 0, 1.0986, 0, -6.5456, 0 - 0.30086, 0, 0
                    , 0, 0, 1.1404, 0, 0, 0.77637, 0, 0, 0, 0, 0
                    , 0, -8.0246, 0, 0, -6.5456, 0, 52.747, 0, 2.0246, 0, 0
                    , 8.2092, 0, 0, 6.6851, 0, 0, 0, 53.919, 0, 2.212, 0
                    , 0, -0.41549, 0, 0, -0.30086, 0, 2.0246, 0, 0.15029, 0, 0
                    , 0.44984, 0, 0, 0.32655, 0, 0, 0, 2.212, 0, 0.1691, 0
                    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.1501;

//            QN.setZero();
//            QN.diagonal() << Q;

            ROS_INFO_STREAM("QN" << QN);

            maxAttitudeAngle = maxAttitudeAngle_degree * (M_PI / 180);

            x_unscaling_vec << scaling_x, scaling_x, scaling_z,
                    max_dx, max_dx, max_dz,
                    1, 1, 1, 1,
                    max_datt, max_datt, max_datt;
            x_scaling_vec = x_unscaling_vec.cwiseInverse();

            u_unscaling_vec << drone->maxServo1Angle, drone->maxServo2Angle,
                    drone->maxPropellerSpeed, drone->maxPropellerDelta / 2;
            u_scaling_vec = u_unscaling_vec.cwiseInverse();

            u_unscaling_vec.setOnes();
            u_scaling_vec.setOnes();
            x_unscaling_vec.setOnes();
            x_scaling_vec.setOnes();

            //scale costs
            x_drone_unscaling_vec = x_unscaling_vec;
            u_drone_unscaling_vec = u_unscaling_vec;

            x_drone_scaling_vec = x_drone_unscaling_vec.cwiseInverse();
            u_drone_scaling_vec = u_drone_unscaling_vec.cwiseInverse();

            Matrix<scalar_t, 11, 1> Q_unscaling_vec;
            Q_unscaling_vec.segment(0, 8) = x_unscaling_vec.segment(0, 8);
            Q_unscaling_vec.segment(8, 3) = x_unscaling_vec.segment(10, 3);
            Q = Q.cwiseProduct(Q_unscaling_vec).cwiseProduct(Q_unscaling_vec);

            Matrix<scalar_t, 11, 11> Q_unscaling_mat;
            Q_unscaling_mat.setZero();
            Q_unscaling_mat.diagonal() << Q_unscaling_vec;
            QN = Q_unscaling_mat * QN * Q_unscaling_mat;

            R = R.cwiseProduct(u_drone_unscaling_vec).cwiseProduct(u_drone_unscaling_vec);
            ROS_INFO_STREAM("u_drone_scale" << u_drone_unscaling_vec);

            ROS_INFO_STREAM("Q" << Q);
            ROS_INFO_STREAM("R" << R);
            ROS_INFO_STREAM("QN" << QN);

            R /= weight_scaling;
            Q /= weight_scaling;
            QN /= weight_scaling;
            R = R.eval();
            Q = Q.eval();
            QN = QN.eval();
        } else {
            ROS_ERROR("Failed to get MPC parameter");
        }

    }

    void getConstraints(state_t <scalar_t> &lbx, state_t <scalar_t> &ubx,
                        control_t <scalar_t> &lbu, control_t <scalar_t> &ubu,
                        constraint_t <scalar_t> &lbg, constraint_t <scalar_t> &ubg) {
        const double inf = std::numeric_limits<double>::infinity();
        const double eps = 1e-1;

        lbu << -drone->maxServo1Angle, -drone->maxServo2Angle,
                drone->minPropellerSpeed, -drone->maxPropellerDelta / 2; // lower bound on control
        ubu << drone->maxServo1Angle, drone->maxServo2Angle,
                drone->maxPropellerSpeed, drone->maxPropellerDelta / 2; // upper bound on control

        lbx << -inf, -inf, min_z + eps,
                -max_dx, -max_dx, min_dz,
                -inf, -inf, -inf, -inf,
                -max_datt, -max_datt, -inf;

        ubx << inf, inf, inf,
                max_dx, max_dx, max_dz,
                inf, inf, inf, inf,
                max_datt, max_datt, inf;

        lbg << drone->minPropellerSpeed, drone->minPropellerSpeed;
        ubg << drone->maxPropellerSpeed, drone->maxPropellerSpeed;
        ROS_INFO_STREAM(lbg);
        ROS_INFO_STREAM("min cos" << cos(maxAttitudeAngle));
//
        lbu = lbu.cwiseProduct(u_scaling_vec);
        ubu = ubu.cwiseProduct(u_scaling_vec);
        ROS_INFO_STREAM("lbu" << lbu.transpose());
        ROS_INFO_STREAM("ubu" << ubu.transpose());

        lbx = lbx.cwiseProduct(x_scaling_vec);
        ubx = ubx.cwiseProduct(x_scaling_vec);
        ROS_INFO_STREAM("lbx" << lbx.transpose());
        ROS_INFO_STREAM("ubx" << ubx.transpose());

    }

    template<typename T>
    inline void dynamics_impl(const Ref<const state_t <T>> x,
                              const Ref<const control_t <T>> u,
                              const Ref<const parameter_t <T>> p,
                              const Ref<const static_parameter_t> &d,
                              const T &t, Ref<state_t < T>>

    xdot)  const noexcept{
        Matrix<T, NX, 1> x_unscaled = x.cwiseProduct(x_unscaling_vec.template cast<T>());
        Matrix<T, NU, 1> u_unscaled = u.cwiseProduct(u_unscaling_vec.template cast<T>());

        Matrix<T, Drone::NP, 1> params;

        drone->getParams(params);

        drone->state_dynamics(x_unscaled, u_unscaled, params, xdot);

        //unscale
        xdot = xdot.cwiseProduct(x_scaling_vec.template cast<T>());

    }

    template<typename T>
    EIGEN_STRONG_INLINE void
    inequality_constraints_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, Ref<constraint_t < T>>

    g) const noexcept
    {
        Matrix<T, 2, 1> u_drone = u.segment(2, 2).cwiseProduct(u_unscaling_vec.segment(2, 2).template cast<T>());;

        g(0) = u_drone(0) + u_drone(1);
        g(1) = u_drone(0) - u_drone(1);
    }

    template<typename T>
    inline void lagrange_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                   const Ref<const parameter_t <T>> p,
                                   const Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        Matrix<T, 13, 1> x_error = x - xs.template cast<T>();
        Matrix<T, 11, 1> x_error2;
        x_error2.segment(0, 6) = x_error.segment(0, 6);
        x_error2(6) = x_error(9) * x_error(6) - x_error(7) * x_error(8);
        x_error2(7) = x_error(9) * x_error(7) + x_error(6) * x_error(8);
        x_error2.segment(8, 3) = x_error.segment(10, 3);

        Matrix<T, NU, 1> u_error = u - us.template cast<T>();


        lagrange = x_error2.dot(Q.template cast<T>().cwiseProduct(x_error2)) +
                   u_error.dot(R.template cast<T>().cwiseProduct(u_error));
//        lagrange = (T) 0;
    }

    template<typename T>
    inline void mayer_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        Matrix<T, 13, 1> x_error = x - xs.template cast<T>();
        Matrix<T, 11, 1> x_error2;
        x_error2.segment(0, 6) = x_error.segment(0, 6);
        x_error2(6) = x_error(9) * x_error(6) - x_error(7) * x_error(8);
        x_error2(7) = x_error(9) * x_error(7) + x_error(6) * x_error(8);
        x_error2.segment(8, 3) = x_error.segment(10, 3);

        mayer = x_error2.dot(QN.template cast<T>() * x_error2);
//        mayer = (T) 0;
//        mayer = p(0);
    }
};

#endif //SRC_DRONE_GUIDANCE_OCP_HPP
