#ifndef SRC_DRONE_GUIDANCE_MPC_H
#define SRC_DRONE_GUIDANCE_MPC_H

#include <drone_model.hpp>

#include <ros/ros.h>
#include "drone_gnc/InterpolateControlSpline.h"

#include "guidance_polympc_redef.hpp"

class DroneGuidanceMPC : public MPC<guidance_ocp, Solver> {

public:
    using ocp_state = state_t;
    using ocp_control = control_t;
    using ocp_constraint = constraint_t;

    DroneGuidanceMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void solve(Drone::state &x0);

    drone_gnc::DroneControl getControlCurrentTime();

    void setTarget(Drone::state &target_state, Drone::control &target_control);

    Drone::state solution_x_at(const double t);
    Drone::control solution_u_at(const double t);
    Drone::state solution_x_at(const int t);
    Drone::control solution_u_at(const int t);
    double node_time(int i);

    void integrateX0(const Drone::state x0, Drone::state &new_x0);
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


#endif //SRC_DRONE_GUIDANCE_MPC_H
