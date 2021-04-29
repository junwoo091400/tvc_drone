#ifndef SRC_DRONE_MPC_H
#define SRC_DRONE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "polympc_redef.hpp"

class DroneMPC {

public:
    using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
            control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;
    using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;
    using mpc_t = MPC<control_ocp, MySolver, admm>;
    using state = mpc_t::state_t;
    using control = mpc_t::control_t;
//    using constraint = mpc_t::constraint_t;

    mpc_t mpc;

    DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void solve(state &x0);

    drone_gnc::DroneControl interpolateControlSplineService();

    void setTarget(state &target_state, control &target_control);

    void integrateX0(const state x0, state &new_x0);
    std::shared_ptr<Drone> drone;
    double mpc_period;
    double feedforward_period;
    double fixed_computation_time;
    double last_computation_time = 0;
    double init_time;
    double horizon_length;

private:
    void warmStart();
    double solution_time;
    bool is_simu;
};


#endif //SRC_DRONE_MPC_H
