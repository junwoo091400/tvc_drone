#ifndef SRC_DRONE_MPC_H
#define SRC_DRONE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "polympc_redef.hpp"
#include "mpc/drone_control_ocp.hpp"

class DroneMPC {

public:
    using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
            control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;
    using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;
    using mpc_t = MPC<control_ocp, MySolver, admm>;
    using state = mpc_t::state_t;
    using control = mpc_t::control_t;
    using constraints = mpc_t::constraints_t;

    mpc_t mpc;

    DroneMPC(ros::NodeHandle &nh);

    void solve(state &x0);

    drone_gnc::DroneControl interpolateControlSplineService();

    void setTarget(state &target_state, control &target_control);

    void integrateX0(const state x0, state &new_x0);
    std::shared_ptr<Drone> drone;
    double mpc_period;
    double feedforward_period;

private:
    void warmStart();

    double solution_time;
};


#endif //SRC_DRONE_MPC_H
