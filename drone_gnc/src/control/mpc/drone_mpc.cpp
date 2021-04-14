#include "drone_mpc.h"

#define CONTROL_HORIZON 2

DroneMPC::DroneMPC(ros::NodeHandle &nh) : solution_time(0) {
    // Initialize rocket class with useful parameters
    drone = make_shared<Drone>();
    drone->init(nh);
    mpc.ocp().init(nh, drone);
    int max_iter, line_search_max_iter;
    nh.getParam("/mpc/max_iter", max_iter);
    nh.getParam("/mpc/line_search_max_iter", line_search_max_iter);
    mpc.settings().max_iter = max_iter;
    mpc.settings().line_search_max_iter = line_search_max_iter;

    // Input constraints
    const double inf = std::numeric_limits<double>::infinity();
    control lbu;
    control ubu;

    lbu << -1, -1, -1, -1; // lower bound on control
    ubu << 1, 1, 1, 1; // upper bound on control

    //lbu << -inf, -inf, -inf, -inf;
    //ubu <<  inf,  inf,  inf,  inf;
    mpc.control_bounds(lbu, ubu);

    // State constraints
    const double eps = 1e-1;
    state lbx;
    state ubx;

    //TODO change state constraints
    lbx << -inf, -inf, -1.0 / 100,
            -inf, -inf, -inf,
            -inf, -inf, -inf, -inf,
            -inf, -inf, -inf;

    ubx << inf, inf, inf,
            inf, inf, inf,
            inf, inf, inf, inf,
            inf, inf, inf;
    mpc.state_bounds(lbx, ubx);
    mpc.set_time_limits(0, CONTROL_HORIZON);
    // Initial state
    state x0;
    state previous_x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    previous_x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    mpc.x_guess(x0.replicate(13, 1));

    control target_control;
    target_control.setZero();

    state target_state;
    target_state.setZero();
}


bool DroneMPC::interpolateControlSplineService(drone_gnc::InterpolateControlSpline::Request &req,
                                               drone_gnc::InterpolateControlSpline::Response &res) {
    if (solution_time != 0) {
        //TODO
        float interp_time = ros::Time::now().toSec() - solution_time;
        ROS_INFO_STREAM(interp_time);

        control interpolated_control = mpc.solution_u_at((double) interp_time);
        drone->unScaleControl(interpolated_control);
        res.drone_control.servo1 = interpolated_control(0);
        res.drone_control.servo2 = interpolated_control(1);
        res.drone_control.bottom = interpolated_control(2);
        res.drone_control.top = interpolated_control(3);

        res.drone_control.servo1 = std::min(std::max(res.drone_control.servo1, -drone->maxServo1Angle),
                                            drone->maxServo1Angle);
        res.drone_control.servo2 = std::min(std::max(res.drone_control.servo2, -drone->maxServo2Angle),
                                            drone->maxServo2Angle);
        res.drone_control.top = std::min(std::max(res.drone_control.top, drone->minPropellerSpeed),
                                         drone->maxPropellerSpeed);
        res.drone_control.bottom = std::min(std::max(res.drone_control.bottom, drone->minPropellerSpeed),
                                            drone->maxPropellerSpeed);
    } else {
        res.drone_control.servo1 = 0;
        res.drone_control.servo2 = 0;
        res.drone_control.bottom = 0;
        res.drone_control.top = 0;
    }

    return true;
}


void DroneMPC::setTarget(state &target_state, control &target_control) {
    mpc.ocp().xs << target_state;
    mpc.ocp().us << target_control;
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

    double time_since_last_solve = ros::Time::now().toSec() - solution_time;

    for (int i = 0; i < NUM_NODES; i++) {
        double interp_time = mpc.time_grid(i) + time_since_last_solve;
        if (interp_time <= CONTROL_HORIZON) {
            x_interp = mpc.solution_x_at(interp_time);
            u_interp = mpc.solution_u_at(interp_time);
        } else {
            drone->stepRK4(x_interp, u_interp, interp_time - previous_interp_time, x_interp);
        }
        x_guess.segment((NUM_NODES - i - 1) * NX, NX) = x_interp;
        u_guess.segment((NUM_NODES - i - 1) * NU, NU) = u_interp;
        previous_interp_time = interp_time;
    }
    mpc.x_guess(x_guess);
    mpc.u_guess(u_guess);
}

void DroneMPC::solve(state &x0) {
    double saved_solution_time = ros::Time::now().toSec();
    mpc.initial_conditions(x0);
    warmStart();
    mpc.solve();
    solution_time = saved_solution_time;
}

