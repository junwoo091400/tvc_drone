#ifndef SRC_DRONE_GUIDANCE_MPC_H
#define SRC_DRONE_GUIDANCE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "guidance_polympc_redef.hpp"

class DroneGuidanceMPC : public MPC<DroneGuidanceOCP, Solver> {

public:
    using ocp_state = state_t;
    using ocp_control = control_t;
    using ocp_constraint = constraint_t;

    Drone::state target_apogee;
    Drone::state target_land;

    DroneGuidanceMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void solve(Drone::state &x0);

    void initGuess(Drone::state &x0, Drone::state &target);

    void setTarget(Drone::state &target_state, Drone::control &target_control);

    Drone::state solution_x_at(const int t);

    Drone::control solution_u_at(const int t);

    double node_time(int i);

    void precomputeDescent();

    void warmStartDescent();

    std::shared_ptr<Drone> drone;
    double last_computation_time = 0;
    double max_horizon_length;

private:
    void warmStart();

    traj_control_t descent_control_sol;
    traj_state_t descent_state_sol;
    dual_var_t descent_dual_sol;
    parameter_t descent_p_sol;
};


#endif //SRC_DRONE_GUIDANCE_MPC_H
