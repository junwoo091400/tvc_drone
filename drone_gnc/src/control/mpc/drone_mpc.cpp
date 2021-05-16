#include "drone_mpc.h"

DroneMPC::DroneMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : solution_time(0), drone(drone_ptr) {
    // Initialize rocket class with useful parameters
    mpc.ocp().init(nh, drone);
    int max_iter, line_search_max_iter;
    nh.getParam("/mpc/max_iter", max_iter);
    nh.getParam("/mpc/line_search_max_iter", line_search_max_iter);
    nh.getParam("/mpc/mpc_period", mpc_period);
    nh.getParam("/mpc/feedforward_period", feedforward_period);
    nh.param("/is_simu", is_simu, true);

    mpc.settings().max_iter = max_iter;
    mpc.settings().line_search_max_iter = line_search_max_iter;
    ROS_INFO_STREAM("max_ter" << line_search_max_iter);

    nh.getParam("/mpc/horizon_length", horizon_length);
    mpc.set_time_limits(0, horizon_length);

    // Setup constraints
    state lbx, ubx; control lbu, ubu; constraint ubg, lbg;
    mpc.ocp().getConstraints(lbx, ubx, lbu, ubu, lbg, ubg);
    mpc.state_bounds(lbx, ubx);
    mpc.control_bounds(lbu, ubu);
    mpc.constraints_bounds(lbg, ubg);

    // Initial state
    state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    mpc.x_guess(x0.cwiseProduct(mpc.ocp().x_scaling_vec).replicate(mpc.ocp().NUM_NODES, 1));
    control u0;
    u0 << 0, 0, drone->getHoverSpeedAverage(), 0;
    mpc.u_guess(u0.cwiseProduct(mpc.ocp().u_scaling_vec).replicate(mpc.ocp().NUM_NODES, 1));

    control target_control;
    target_control.setZero();

    state target_state;
    target_state.setZero();

    init_time = ros::Time::now().toSec();
    fixed_computation_time = mpc_period*0.98;
}


drone_gnc::DroneControl DroneMPC::getControlCurrentTime() {
    double interp_time = ros::Time::now().toSec() - solution_time;
    interp_time = std::max(std::min(interp_time, horizon_length), 0.0);
    control interpolated_control = mpc.solution_u_at(interp_time).cwiseProduct(mpc.ocp().u_unscaling_vec);

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


void DroneMPC::setTarget(state &target_state, control &target_control) {
    mpc.ocp().xs << target_state.cwiseProduct(mpc.ocp().x_scaling_vec);
    mpc.ocp().us << target_control.cwiseProduct(mpc.ocp().u_scaling_vec);
}

void DroneMPC::warmStart() {
    //warm start
    mpc_t::traj_state_t x_guess;
    mpc_t::traj_control_t u_guess;
    int NX = mpc.ocp().NX;
    int NU = mpc.ocp().NU;
    int NUM_NODES = mpc.ocp().NUM_NODES;
    state x_interp;
    control u_interp;
    double previous_interp_time = 0;

    double time_since_last_solve = feedforward_period;

    for (int i = 0; i < NUM_NODES; i++) {
        double interp_time = mpc.time_grid(i) + time_since_last_solve;
        if (interp_time <= horizon_length) {
            x_interp = mpc.solution_x_at(interp_time);
            u_interp = mpc.solution_u_at(interp_time);
        } else {
//            drone->stepRK4(x_interp.cwiseProduct(mpc.ocp().x_unscaling_vec),
//                           u_interp.cwiseProduct(mpc.ocp().u_unscaling_vec),
//                           interp_time - previous_interp_time,
//                           x_interp);
//            x_interp = x_interp.cwiseProduct(mpc.ocp().x_scaling_vec);
        }
        x_guess.segment((NUM_NODES - i - 1) * NX, NX) = x_interp;
        u_guess.segment((NUM_NODES - i - 1) * NU, NU) = u_interp;
        previous_interp_time = interp_time;
    }
    mpc.x_guess(x_guess);
    mpc.u_guess(u_guess);
}

void DroneMPC::solve(state &x0) {
    double computation_start_time = ros::Time::now().toSec();
//    ROS_INFO_STREAM("x0     " <<x0.transpose().head(6)*100);

    state predicted_x0;
    integrateX0(x0, predicted_x0);

//    ROS_INFO_STREAM("predicted x0 " <<predicted_x0.transpose().head(6)*100);

    mpc.initial_conditions(predicted_x0.cwiseProduct(mpc.ocp().x_scaling_vec));
//    warmStart();


    if (is_simu){
        // in simulation, sleep to account fo
        ros::Duration(mpc_period-feedforward_period).sleep();
    }

    double time_now = ros::Time::now().toSec();
    mpc.solve();
    last_computation_time = (ros::Time::now().toSec()-time_now)*1000;

    time_now = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < computation_start_time+fixed_computation_time);
    ROS_INFO_STREAM("stalled duration " << (ros::Time::now().toSec() - time_now)*1000);

    solution_time = ros::Time::now().toSec();
}

void DroneMPC::integrateX0(const state x0, state &new_x0){
    new_x0 = x0;

    int max_ff_index = std::round(mpc_period/feedforward_period);

    int i;
    for(i = 0; i<max_ff_index-1; i++){
        control interpolated_control = mpc.solution_u_at(feedforward_period*i).cwiseProduct(mpc.ocp().u_unscaling_vec);
        drone->stepRK4(x0, interpolated_control, mpc_period, new_x0);
    }
    i++;

    control interpolated_control = mpc.solution_u_at(feedforward_period*i).cwiseProduct(mpc.ocp().u_unscaling_vec);
    drone->stepRK4(x0, interpolated_control, mpc_period, new_x0);
}

