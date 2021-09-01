
#ifndef SRC_DRONE_GUIDANCE_OCP_HPP
#define SRC_DRONE_GUIDANCE_OCP_HPP

#include "ros/ros.h"

#include <string>
#include <math.h>

#include <chrono>

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/osqp_interface.hpp"
#include "control/mpc_wrapper.hpp"
#include "drone_model.hpp"

using namespace Eigen;

// Poly MPC stuff ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

using namespace std;

#define POLY_ORDER 7
#define NUM_SEG    2

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ DroneGuidanceOCP, /*NX*/ 6, /*NU*/ 3, /*NP*/ 1, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)

class DroneGuidanceOCP : public ContinuousOCP<DroneGuidanceOCP, Approximation, SPARSE> {
public:
    ~DroneGuidanceOCP() = default;

    Matrix<scalar_t, NU, 1> R;

    Matrix<scalar_t, NX, 1> xs;
    Matrix<scalar_t, NU, 1> us;

    shared_ptr<Drone> drone;

    scalar_t min_z;
    scalar_t min_dz;
    scalar_t max_dx;
    scalar_t max_dz;
    scalar_t scaling_x;
    scalar_t scaling_z;

    Matrix<scalar_t, NX, 1> x_unscaling_vec;
    Matrix<scalar_t, NU, 1> u_unscaling_vec;

    Matrix<scalar_t, NX, 1> x_scaling_vec;
    Matrix<scalar_t, NU, 1> u_scaling_vec;

    scalar_t max_fx;
    scalar_t horizontal_slack;

    void init(ros::NodeHandle nh, shared_ptr<Drone> drone_ptr) {
        drone = drone_ptr;
        scalar_t thrust_cost, lateral_force;
        scalar_t weight_scaling;
        if (nh.getParam("mpc/min_z", min_z) &&
            nh.getParam("mpc/min_dz", min_dz) &&
            nh.getParam("mpc/max_dx", max_dx) &&
            nh.getParam("mpc/max_dz", max_dz) &&
            nh.getParam("mpc/scaling_x", scaling_x) &&
            nh.getParam("mpc/scaling_z", scaling_z) &&
            nh.getParam("mpc/weight_scaling", weight_scaling) &&
            nh.getParam("mpc/max_fx", max_fx) &&
            nh.getParam("mpc/input_costs/thrust", thrust_cost) &&
            nh.getParam("mpc/input_costs/lateral_force", lateral_force) &&
            nh.getParam("mpc/horizontal_slack", horizontal_slack)) {

            x_unscaling_vec << scaling_x, scaling_x, scaling_z,
                    max_dx, max_dx, max_dz;
            x_scaling_vec = x_unscaling_vec.cwiseInverse();

            u_unscaling_vec << max_fx, max_fx,
                    drone->max_propeller_speed;
            u_scaling_vec = u_unscaling_vec.cwiseInverse();

            R << thrust_cost, lateral_force, lateral_force;

            u_unscaling_vec.setOnes();
            u_scaling_vec.setOnes();
            x_unscaling_vec.setOnes();
            x_scaling_vec.setOnes();

        } else {
            ROS_ERROR("Failed to get Guidance MPC parameter");
        }

    }

    void get_state_bounds(state_t <scalar_t> &lbx, state_t <scalar_t> &ubx) {
        const double inf = std::numeric_limits<double>::infinity();
        const double eps = 1e-1;

        lbx << -inf, -inf, 0 - eps,
                -max_dx, -max_dx, min_dz;

        ubx << inf, inf, inf,
                max_dx, max_dx, max_dz;

        lbx = lbx.cwiseProduct(x_scaling_vec);
        ubx = ubx.cwiseProduct(x_scaling_vec);
    }


    void get_control_bounds(control_t <scalar_t> &lbu, control_t <scalar_t> &ubu) {
        lbu << -max_fx, -max_fx,
                drone->min_propeller_speed; // lower bound on control
        ubu << max_fx, max_fx,
                drone->max_propeller_speed; // upper bound on control

        lbu = lbu.cwiseProduct(u_scaling_vec);
        ubu = ubu.cwiseProduct(u_scaling_vec);
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

//        Matrix<T, Drone::NP, 1> params;
//        drone->getParams(params);

        T fx = u_unscaled(0);
        T fy = u_unscaled(1);
        T prop_av = u_unscaled(2);

        T thrust = drone->getThrust(prop_av) * drone->thrust_scaling;

        Eigen::Matrix<T, 3, 1> thrust_vector;
        thrust_vector << fx, fy, thrust;

        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T) 0, (T) 0, (T) - 9.81;

        xdot.head(3) = x_unscaled.segment(3, 3);
        xdot.segment(3, 3) = thrust_vector * (T) drone->dry_mass_inv + gravity;

        //unscale
        xdot = xdot.cwiseProduct(x_scaling_vec.template cast<T>());

        //minimal time
        xdot *= p(0);
    }

    template<typename T>
    EIGEN_STRONG_INLINE void
    inequality_constraints_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, Ref<constraint_t < T>>

    g) const noexcept
    {
//        Matrix<T, 2, 1> u_drone = u.segment(2, 2).cwiseProduct(u_unscaling_vec.segment(2, 2).template cast<T>());;
//
//        g(0) = u_drone(0) + u_drone(1);
//        g(1) = u_drone(0) - u_drone(1);
    }

    template<typename T>
    inline void lagrange_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                   const Ref<const parameter_t <T>> p,
                                   const Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        lagrange = p(0)*(u.dot(R.template cast<T>().cwiseProduct(u)));
    }

    template<typename T>
    inline void mayer_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        mayer = horizontal_slack*((x(0) - xs(0))*(x(0) - xs(0)) + (x(1) - xs(1))*(x(1) - xs(1)) );
    }
};

#endif //SRC_DRONE_GUIDANCE_OCP_HPP
