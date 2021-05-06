
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

time_point get_time() {
    /** OS dependent */
#ifdef __APPLE__
    return std::chrono::system_clock::now();
#else
    return std::chrono::high_resolution_clock::now();
#endif
}

#define POLY_ORDER 5
#define NUM_SEG    2
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

    scalar_t attitude_cost;

    shared_ptr<Drone> drone;

    scalar_t maxAttitudeAngle;
    scalar_t maxAttitudeCos;

    void init(ros::NodeHandle nh, shared_ptr<Drone> d) {
        scalar_t x_cost, dx_cost, z_cost, dz_cost, att_cost, datt_cost, servo_cost, thrust_cost, torque_cost, droll_cost;
        if (nh.getParam("/mpc/state_costs/x", x_cost) &&
            nh.getParam("/mpc/state_costs/dz", dx_cost) &&
            nh.getParam("/mpc/state_costs/z", z_cost) &&
            nh.getParam("/mpc/state_costs/dz", dz_cost) &&
            nh.getParam("/mpc/state_costs/att", att_cost) &&
            nh.getParam("/mpc/state_costs/datt", datt_cost) &&
            nh.getParam("/mpc/state_costs/droll", droll_cost) &&
            nh.getParam("/mpc/input_costs/servo", servo_cost) &&
            nh.getParam("/mpc/input_costs/thrust", thrust_cost) &&
            nh.getParam("/mpc/input_costs/torque", torque_cost) &&
            nh.getParam("/rocket/maxAttitudeAngle", maxAttitudeAngle)) {

//            Q << x_cost, x_cost, z_cost,
//                    dx_cost, dx_cost, dz_cost,
//                    0, 0, 0, 0,
//                    datt_cost, datt_cost, droll_cost;
            Q << 1, 1, 1,
                    1, 1, 1,
                    1, 1, 1, 0,
                    1, 1, 1;
            R << 300, 300, 0.1, 0.01;
//            QN.setZero();
//            QN.diagonal() << Q*50;
            QN << 87.783, 0, 0, 51.181, 0, 0, 0, 327.73, 0, 0, 0, 29.712, 0,
                    0, 87.025, 0, 0, 49.862, 0, -312.84, 0, 0, 0, -27.409, 0, 0,
                    0, 0, 99.806, 0, 0, 73.614, 0, 0, 0, 0, 0, 0, 0,
                    51.181, 0, 0, 73.141, 0, 0, 0, 512.62, 0, 0, 0, 48.305, 0,
                    0, 49.862, 0, 0, 70.842, 0, -486.48, 0, 0, 0, -44.265, 0, 0,
                    0, 0, 73.614, 0, 0, 146.71, 0, 0, 0, 0, 0, 0, 0,
                    0, -312.84, 0, 0, -486.48, 0, 4383.7, 0, 0, 0, 439.18, 0, 0,
                    327.73, 0, 0, 512.62, 0, 0, 0, 4686.1, 0, 0, 0, 486.88, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 140.56, 0, 0, 0, 48.08,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, -27.409, 0, 0, -44.265, 0, 439.18, 0, 0, 0, 64.052, 0, 0,
                    29.712, 0, 0, 48.305, 0, 0, 0, 486.88, 0, 0, 0, 72.401, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 48.08, 0, 0, 0, 67.839;
//            scaleCost(Q);
//            scaleCost(QN);


            attitude_cost = att_cost;
            maxAttitudeCos = cos(maxAttitudeAngle);

            drone = d;
        } else {
            ROS_ERROR("Failed to get MPC parameter");
        }

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

        //cos of angle from vertical
//        T attitude_error = x(9) * x(9) - x(6) * x(6) - x(7) * x(7) + x(8) * x(8);

        lagrange = x_error.dot(Q.template cast<T>().cwiseProduct(x_error)) +
                   //                   attitude_cost * attitude_error +
                   u_error.dot(R.template cast<T>().cwiseProduct(u_error));
    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                                const Eigen::Ref<const parameter_t <T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        Eigen::Matrix<T, NX, 1> x_error = x - xs.template cast<T>();

        //cos of angle from vertical
//        T attitude_error = x(9) * x(9) - x(6) * x(6) - x(7) * x(7) + x(8) * x(8);

        mayer = x_error.dot(QN.template cast<T>() * x_error);
//                attitude_cost * attitude_error;
    }
};

#endif //SRC_DRONE_CONTROL_OCP_HPP
