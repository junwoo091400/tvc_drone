#include "drone_mpc.h"

DroneMPC::DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : solution_time(0), drone(drone_ptr) {
    // Initialize rocket class with useful parameters
    ocp().init(nh, drone);
    int max_sqp_iter, max_qp_iter, max_line_search_iter;
    nh.getParam("mpc/max_sqp_iter", max_sqp_iter);
    nh.getParam("mpc/max_line_search_iter", max_line_search_iter);
    nh.getParam("mpc/mpc_period", mpc_period);
    nh.getParam("mpc/feedforward_period", feedforward_period);
    nh.getParam("mpc/max_qp_iter", max_qp_iter);
    nh.param("/is_simu", is_simu, true);

    settings().max_iter = max_sqp_iter;
    settings().line_search_max_iter = max_line_search_iter;
    qp_settings().max_iter = max_qp_iter;

    nh.getParam("mpc/horizon_length", horizon_length);
    set_time_limits(0, horizon_length);

    // Setup constraints
    ocp_state lbx, ubx; ocp_control lbu, ubu; ocp_constraint ubg, lbg;
    ocp().getConstraints(lbx, ubx, lbu, ubu, lbg, ubg);
    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);
    constraints_bounds(lbg, ubg);

    // Initial state
    ocp_state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0,
            0, 0;
    x_guess(x0.cwiseProduct(ocp().x_scaling_vec).replicate(ocp().NUM_NODES, 1));
    ocp_control u0;
    u0 << 0, 0, drone->getHoverSpeedAverage(), 0;
    u_guess(u0.cwiseProduct(ocp().u_scaling_vec).replicate(ocp().NUM_NODES, 1));

    Drone::control target_control;
    target_control << 0, 0, drone->getHoverSpeedAverage(), 0;
    Drone::state target_state;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    setTarget(target_state, target_control);

    init_time = ros::Time::now().toSec();
    fixed_computation_time = mpc_period*0.93;
}


drone_gnc::DroneControl DroneMPC::getControlCurrentTime() {
    double interp_time = ros::Time::now().toSec() - solution_time;
    interp_time = std::max(std::min(interp_time, horizon_length), 0.0);
    Drone::control interpolated_control = solution_u_at(interp_time);

    drone_gnc::DroneControl drone_control;
    drone_control.servo1 = interpolated_control(0);
    drone_control.servo2 = interpolated_control(1);
    drone_control.bottom = interpolated_control(2) - interpolated_control(3)/2;
    drone_control.top = interpolated_control(2) + interpolated_control(3)/2;

    drone_control.servo1 = std::min(std::max(drone_control.servo1, -drone->maxServo1Angle),
                                        drone->maxServo1Angle);
    drone_control.servo2 = std::min(std::max(drone_control.servo2, -drone->maxServo2Angle),
                                        drone->maxServo2Angle);
    drone_control.top = std::min(std::max(drone_control.top, drone->minPropellerSpeed),
                                     drone->maxPropellerSpeed);
    drone_control.bottom = std::min(std::max(drone_control.bottom, drone->minPropellerSpeed),
                                        drone->maxPropellerSpeed);
    return drone_control;
}


void DroneMPC::setTarget(Drone::state &target_state, Drone::control &target_control) {
    //TODO
//    ocp().xs << target_state.cwiseProduct(ocp().x_drone_scaling_vec);
    ocp().us << target_control.cwiseProduct(ocp().u_drone_scaling_vec);
}

void DroneMPC::warmStart() {
//    //warm start
//    traj_state_t x_guess;
//    traj_control_t u_guess;
//    int NX = ocp().NX;
//    int NU = ocp().NU;
//    int NUM_NODES = ocp().NUM_NODES;
//    state x_interp;
//    control u_interp;
//    double previous_interp_time = 0;
//
//    double time_since_last_solve = feedforward_period;
//
//    for (int i = 0; i < NUM_NODES; i++) {
//        double interp_time = time_grid(i) + time_since_last_solve;
//        if (interp_time <= horizon_length) {
//            x_interp = solution_x_at(interp_time);
//            u_interp = solution_u_at(interp_time);
//        } else {
////            drone->stepRK4(x_interp.cwiseProduct(mpc.ocp().x_unscaling_vec),
////                           u_interp.cwiseProduct(mpc.ocp().u_unscaling_vec),
////                           interp_time - previous_interp_time,
////                           x_interp);
////            x_interp = x_interp.cwiseProduct(mpc.ocp().x_scaling_vec);
//        }
//        x_guess.segment((NUM_NODES - i - 1) * NX, NX) = x_interp;
//        u_guess.segment((NUM_NODES - i - 1) * NU, NU) = u_interp;
//        previous_interp_time = interp_time;
//    }
//    x_guess(x_guess);
//    u_guess(u_guess);
}

Drone::state DroneMPC::solution_x_at(const double t){
    return MPC::solution_x_at(t).segment(0, 13).cwiseProduct(ocp().x_drone_unscaling_vec);
}

Drone::control DroneMPC::solution_u_at(const double t){
    Drone::control u;
    u.segment(0, 2) =  MPC::solution_x_at(t).segment(13, 2);
    u.segment(2, 2) =  MPC::solution_u_at(t).segment(2, 2);
    return u.cwiseProduct(ocp().u_drone_unscaling_vec);
}

Drone::state DroneMPC::solution_x_at(const int t){
    return MPC::solution_x_at(t).segment(0, 13).cwiseProduct(ocp().x_drone_unscaling_vec);
}

Drone::control DroneMPC::solution_u_at(const int t){
    Drone::control u;
    u.segment(0, 2) =  MPC::solution_x_at(t).segment(13, 2);
    u.segment(2, 2) =  MPC::solution_u_at(t).segment(2, 2);
    return u.cwiseProduct(ocp().u_drone_unscaling_vec);
}

double DroneMPC::node_time(int i){
    return time_grid(i);
}

void DroneMPC::solve(Drone::state &x0) {
    double computation_start_time = ros::Time::now().toSec();

    Drone::state predicted_x0;
    integrateX0(x0, predicted_x0);

    //servo rate constraint
    ocp_state previous_x = MPC::solution_x_at(0).cwiseProduct(ocp().x_unscaling_vec);
    double maxServoRate = drone->maxServoRate;
    ocp_state lbx0;
    lbx0.segment(0, 13) = predicted_x0;
    lbx0(13) = previous_x(13) - maxServoRate*mpc_period;
    lbx0(14) = previous_x(14) - maxServoRate*mpc_period;

    ocp_state ubx0;
    ubx0.segment(0, 13) = predicted_x0;
    ubx0(13) = previous_x(13) + maxServoRate*mpc_period;
    ubx0(14) = previous_x(14) + maxServoRate*mpc_period;

    initial_conditions(lbx0.cwiseProduct(ocp().x_scaling_vec), ubx0.cwiseProduct(ocp().x_scaling_vec));

//    warmStart();

    if (is_simu){
        // in simulation, sleep to account fo
        ros::Duration(mpc_period-feedforward_period).sleep();
    }

    double time_now = ros::Time::now().toSec();
    MPC::solve();
    last_computation_time = (ros::Time::now().toSec()-time_now)*1000;

    time_now = ros::Time::now().toSec();
    //TODO
    while (ros::Time::now().toSec() < computation_start_time+fixed_computation_time);
    ROS_INFO_STREAM("");

    solution_time = ros::Time::now().toSec();
}

void DroneMPC::integrateX0(const Drone::state x0, Drone::state &new_x0){
    new_x0 = x0;

    int max_ff_index = std::round(mpc_period/feedforward_period);

    int i;
    for(i = 0; i<max_ff_index-1; i++){
        Drone::control interpolated_control = solution_u_at(feedforward_period*i);
        drone->stepRK4(x0, interpolated_control, mpc_period, new_x0);
    }
    i++;

    Drone::control interpolated_control = solution_u_at(feedforward_period*i);
    drone->stepRK4(x0, interpolated_control, mpc_period, new_x0);
}

