
#ifndef SRC_DRONE_CONTROL_OCP_HPP
#define SRC_DRONE_CONTROL_OCP_HPP

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

typedef std::chrono::time_point<std::chrono::system_clock> time_point;

//time_point get_time() {
//    /** OS dependent */
//#ifdef __APPLE__
//    return std::chrono::system_clock::now();
//#else
//    return std::chrono::high_resolution_clock::now();
//#endif
//}

#define POLY_ORDER 7
#define NUM_SEG    1
#define NUM_EXP    1

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ control_ocp, /*NX*/ 13, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/3, /*TYPE*/ double)

class control_ocp : public ContinuousOCP<control_ocp, Approximation, SPARSE> {
public:
    ~control_ocp() = default;

    Eigen::Matrix<scalar_t, NX, 1> Q;
    Eigen::Matrix<scalar_t, NU, 1> R;
    Eigen::Matrix<scalar_t, NX, NX> QN;

    Eigen::Matrix<scalar_t, NX, 1> xs;
    Eigen::Matrix<scalar_t, NU, 1> us;

    shared_ptr<Drone> drone;

    scalar_t maxAttitudeAngle;

    scalar_t min_z;
    scalar_t min_dz;
    scalar_t max_dx;
    scalar_t max_dz;
    scalar_t max_datt;
    scalar_t scaling_x;
    scalar_t scaling_z;

    void init(ros::NodeHandle nh, shared_ptr<Drone> d) {
//        scalar_t x_cost, dx_cost, z_cost, dz_cost, att_cost, datt_cost, servo_cost, thrust_cost, torque_cost, droll_cost;
        scalar_t maxAttitudeAngle_degree;
        if (nh.getParam("/mpc/min_z", min_z) &&
            nh.getParam("/mpc/min_dz", min_dz) &&
            nh.getParam("/mpc/max_dx", max_dx) &&
            nh.getParam("/mpc/max_dz", max_dz) &&
            nh.getParam("/mpc/max_datt", max_datt) &&
            nh.getParam("/mpc/scaling_x", scaling_x) &&
            nh.getParam("/mpc/scaling_z", scaling_z) &&

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

            nh.getParam("/mpc/max_attitude_angle", maxAttitudeAngle_degree)) {

//            Q << x_cost, x_cost, z_cost,
//                    dx_cost, dx_cost, dz_cost,
//                    0, 0, 0, 0,
//                    datt_cost, datt_cost, droll_cost;
            Q << 1, 1, 1,
                    0.1, 0.1, 0.1,
                    1, 1, 1, 0,
                    1, 1, 1;
            R << 5, 5, 0.01, 0.01;

            QN << 1.0862, 0, 0, 0.53991, 0, 0, 0, 2.9102, 0, 0, 0, 0.16346, 0
                    , 0, 1.0808, 0, 0, 0.53405, 0, -2.8447, 0, 0, 0, -0.15106, 0, 0
                    , 0, 0, 1.0154, 0, 0, 0.46557, 0, 0, 0, 0, 0, 0, 0
                    , 0.53991, 0, 0, 0.43811, 0, 0, 0, 2.8341, 0, 0, 0, 0.14597, 0
                    , 0, 0.53405, 0, 0, 0.4322, 0, -2.7724, 0, 0, 0, -0.13452, 0, 0
                    , 0, 0, 0.46557, 0, 0, 0.47276, 0, 0, 0, 0, 0, 0, 0
                    , 0, -2.8447, 0, 0, -2.7724, 0, 24.528, 0, 0, 0, 1.0231, 0, 0
                    , 2.9102, 0, 0, 2.8341, 0, 0, 0, 25.1, 0, 0, 0, 1.1173, 0
                    , 0, 0, 0, 0, 0, 0, 0, 0, 2.8011, 0, 0, 0, 0.96154
                    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    , 0, -0.15106, 0, 0, -0.13452, 0, 1.0231, 0, 0, 0, 0.090937, 0, 0
                    , 0.16346, 0, 0, 0.14597, 0, 0, 0, 1.1173, 0, 0, 0, 0.10168, 0
                    , 0, 0, 0, 0, 0, 0, 0, 0, 0.96154, 0, 0, 0, 1.3467;


            ROS_INFO_STREAM("QN" << QN);
//            scaleCost(Q);
//            scaleCost(QN);

            maxAttitudeAngle = maxAttitudeAngle_degree * (M_PI / 180);

            drone = d;
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
                drone->minPropellerSpeed, -drone->maxPropellerDelta; // lower bound on control
        ubu << drone->maxServo1Angle, drone->maxServo2Angle,
                drone->maxPropellerSpeed, drone->maxPropellerDelta; // upper bound on control

        //TODO change state constraints
        lbx << -inf, -inf, min_z + eps,
                -max_dx, -max_dx, min_dz,
                -inf, -inf, -inf, -inf,
                -max_datt, -max_datt, -inf;

        ubx << inf, inf, inf,
                max_dx, max_dx, max_dz,
                inf, inf, inf, inf,
                max_datt, max_datt, inf;

        ROS_INFO_STREAM(lbx.transpose());
        ROS_INFO_STREAM(ubx.transpose());

        //TODO fix attitude constraint [cos(maxAttitudeAngle) 1]
        lbg << drone->minPropellerSpeed, drone->minPropellerSpeed, -inf;
        ubg << drone->maxPropellerSpeed, drone->maxPropellerSpeed, inf;
        ROS_INFO_STREAM(lbg);

        scaleState(lbx);
        scaleState(ubx);
    }


    template<typename T>
    inline void unScaleControl(control_t <T> &u) const {
//        u(0) = drone->maxServo1Angle * u(0);
//        u(1) = drone->maxServo2Angle * u(1);
//        u(2) = 0.5 * (u(2) + 1) * (drone->maxPropellerSpeed - drone->minPropellerSpeed) + drone->minPropellerSpeed;
//        u(3) = u(3) * drone->maxPropellerDelta;
    }


    inline void scaleControl(control_t<double> &u) const {
//        u(0) = u(0) / drone->maxServo1Angle;
//        u(1) = u(1) / drone->maxServo2Angle;
//        u(2) = 2 * (u(2) - drone->minPropellerSpeed) / (drone->maxPropellerSpeed - drone->minPropellerSpeed) - 1;
//        u(3) = u(3) / drone->maxPropellerDelta;
    }

    inline void scaleState(state_t<double> &x) const {
        x.segment(0, 6) *= 1;
    }

    inline void unScaleState(state_t<double> &x) const {
        x.segment(0, 6) *= 1;
    }

    inline void scaleCost(state_t<double> &Q) const {
        Q.segment(0, 6) *= 1;
    }

    template<typename T>
    inline void scaleStateDerivative(Eigen::Ref<state_t < T>>

    xdot) const {
        xdot.segment(3, 3) *= (T) 1;
    }

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t <T>> x,
                              const Eigen::Ref<const control_t <T>> u,
                              const Eigen::Ref<const parameter_t <T>> p,
                              const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t < T>>

    xdot)  const noexcept{
        Eigen::Matrix<T, Drone::NX, 1> x_drone = x.segment(0, Drone::NX);

        Eigen::Matrix<T, Drone::NU, 1> u_drone = u.segment(0, Drone::NU);

        Eigen::Matrix<T, Drone::NP, 1> params;

        drone->getParams(params);

        unScaleControl(u_drone);
        drone->state_dynamics(x_drone, u_drone, params, xdot);
        scaleStateDerivative(xdot);
    }

    template<typename T>
    EIGEN_STRONG_INLINE void
    inequality_constraints_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                                const Eigen::Ref<const parameter_t <T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, Eigen::Ref<constraint_t < T>>

    g) const noexcept
    {
        T attitude_error_cos = x(9) * x(9) - x(6) * x(6) - x(7) * x(7) + x(8) * x(8);
        Eigen::Matrix<T, 4, 1> u_drone = u.segment(0, 4);
        unScaleControl(u_drone);

        g(0) = u_drone(2) + u_drone(3);
        g(1) = u_drone(2) - u_drone(3);
        g(2) = attitude_error_cos;
    }

    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                                   const Eigen::Ref<const parameter_t <T>> p,
                                   const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        Eigen::Matrix<T, NX, 1> x_error = x - xs.template cast<T>();
        Eigen::Matrix<T, NU, 1> u_error = u - us.template cast<T>();

        lagrange = x_error.dot(Q.template cast<T>().cwiseProduct(x_error)) +
                   u_error.dot(R.template cast<T>().cwiseProduct(u_error));
    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                                const Eigen::Ref<const parameter_t <T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        Eigen::Matrix<T, NX, 1> x_error = x - xs.template cast<T>();

        mayer = x_error.dot(QN.template cast<T>() * x_error);
    }
};

#endif //SRC_DRONE_CONTROL_OCP_HPP
