#ifndef SRC_DRONE_MPC_H
#define SRC_DRONE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "polympc_redef.hpp"

class DroneMPC : public MPC<control_ocp, Solver> {

public:
//    using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
//            control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;
//    using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;
    using state = state_t;
    using control = control_t;
    using constraint = constraint_t;

    DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void solve(state &x0);

    drone_gnc::DroneControl getControlCurrentTime();

    void setTarget(state &target_state, control &target_control);

    state solution_x_at(const double t);
    control solution_u_at(const double t);
    state solution_x_at(const int t);
    control solution_u_at(const int t);
    double node_time(int i);

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
