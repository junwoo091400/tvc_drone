#include "drone_guidance_mpc.h"


DroneGuidanceMPC::DroneGuidanceMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone(drone_ptr) {
    // Initialize rocket class with useful parameters
    ocp().init(nh, drone);
    int max_sqp_iter, max_qp_iter, max_line_search_iter;
    nh.getParam("mpc/max_sqp_iter", max_sqp_iter);
    nh.getParam("mpc/max_line_search_iter", max_line_search_iter);
    nh.getParam("mpc/mpc_period", mpc_period);
    nh.getParam("mpc/feedforward_period", feedforward_period);
    nh.getParam("mpc/max_qp_iter", max_qp_iter);

    settings().max_iter = max_sqp_iter;
    settings().line_search_max_iter = max_line_search_iter;
    qp_settings().max_iter = max_qp_iter;

    nh.getParam("mpc/horizon_length", horizon_length);
    if (ocp().minimal_time) {
        set_time_limits(0, 1);
    } else {
        set_time_limits(0, horizon_length);
    }

    // Setup constraints
    ocp_state lbx, ubx;
    ocp_control lbu, ubu;
    ocp().get_control_bounds(lbu, ubu);
    ocp().get_state_bounds(lbx, ubx);

    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);

    //minimal time setup
    parameter_t lbp;
    parameter_t ubp;
    if (ocp().minimal_time) {
        lbp << 0.1;
        ubp << horizon_length;
    } else {
        lbp << 1.0;
        ubp << 1.0;
    }
    parameters_bounds(lbp, ubp);


    Drone::control target_control;
    target_control << 0, 0, drone->getHoverSpeedAverage(), 0;
    Drone::state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    std::vector<double> initial_target_apogee;
    nh.getParam("target_apogee", initial_target_apogee);
    Drone::state target_state;
    target_state << initial_target_apogee.at(0), initial_target_apogee.at(1), initial_target_apogee.at(2),
    0, 0, 0,
    0, 0, 0, 1,
    0, 0, 0;

    setTarget(target_state, target_control);
    initGuess(x0, target_state);

    fixed_computation_time = mpc_period * 0.93;
}

void DroneGuidanceMPC::initGuess(Drone::state &x0, Drone::state &target_state){
    // Initial guess
    double z0 = x0(2);
    double z_target = target_state(2);

    parameter_t p0;
    Eigen::MatrixXd traj_state_guess = x0.head(6).replicate(1, ocp().NUM_NODES);

    ocp_control u0;
    u0 << 0, 0, drone->maxPropellerSpeed;
    Eigen::MatrixXd traj_control_guess = u0.replicate(1, ocp().NUM_NODES);

    if (ocp().minimal_time) {
        double a1 = drone->getThrust(drone->maxPropellerSpeed) / drone->dry_mass - 9.81;
        double a2 = drone->getThrust(drone->minPropellerSpeed) / drone->dry_mass - 9.81;
        double t_mid = sqrt(2 * (z_target-z0) / (a1 * (1 - a1 / a2)));
        double t_end = t_mid-a1 / a2 * t_mid;
        double z_mid = 0.5 * a1 * t_mid * t_mid;
        double v_mid = a1 * t_mid;

        RowVectorXd z_guess(ocp().NUM_NODES);
        RowVectorXd dz_guess(ocp().NUM_NODES);
        RowVectorXd prop_av_guess(ocp().NUM_NODES);
        for (int i = 0; i < time_grid.size(); i++) {
            double t = time_grid(i) * t_end;
            if (t < t_mid) {
                z_guess(i) = 0.5 * a1 * t * t;
                dz_guess(i) = a1 * t;
                prop_av_guess(i) = drone->maxPropellerSpeed;
            } else {
                z_guess(i) = z_mid + v_mid * (t - t_mid) + 0.5 * a2 * (t - t_mid) * (t - t_mid);
                dz_guess(i) = v_mid + a2 * (t - t_mid);
                prop_av_guess(i) = drone->minPropellerSpeed;
            }
        }

        traj_state_guess.row(2) = z_guess.reverse();
        traj_state_guess.row(5) = dz_guess.reverse();
        traj_control_guess.row(2) = prop_av_guess.reverse();
//        ROS_INFO_STREAM("traj guess " << traj_state_guess);
//        ROS_INFO_STREAM("control guess " << traj_control_guess);

        p0 << t_end;
    } else {
        p0 << 1.0;
    }
    p_guess(p0);

    //TODO scaling
    traj_state_guess.resize(varx_size, 1);
    x_guess(traj_state_guess);

    traj_control_guess.resize(varu_size, 1);
    u_guess(traj_control_guess);

    dual_var_t dual;
    dual.setZero();
    lam_guess(dual);
}

void DroneGuidanceMPC::setTarget(Drone::state &target_state, Drone::control &target_control) {
    ocp().xs << target_state.head(6).cwiseProduct(ocp().x_scaling_vec);
    ocp().us << target_control.head(3).cwiseProduct(ocp().u_scaling_vec);
}

void DroneGuidanceMPC::warmStart() {
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

Drone::state DroneGuidanceMPC::solution_x_at(const double t) {
    Drone::state sol;
    sol << MPC::solution_x_at(t).cwiseProduct(ocp().x_unscaling_vec),
            0, 0, 0, 1,
            0, 0, 0;
    return sol;
}

Drone::control DroneGuidanceMPC::solution_u_at(const double t) {
    Drone::control sol;
    sol << MPC::solution_u_at(t).cwiseProduct(ocp().u_unscaling_vec), 0;
    return sol;
}

Drone::state DroneGuidanceMPC::solution_x_at(const int t) {
    Drone::state sol;
    sol << MPC::solution_x_at(t).cwiseProduct(ocp().x_unscaling_vec),
            0, 0, 0, 1,
            0, 0, 0;
    return sol;
}

Drone::control DroneGuidanceMPC::solution_u_at(const int t) {
    Drone::control sol;
    sol << MPC::solution_u_at(t).cwiseProduct(ocp().u_unscaling_vec), 0;
    return sol;
}

double DroneGuidanceMPC::node_time(int i) {
    return time_grid(i);
}

void DroneGuidanceMPC::solve(Drone::state &x0_full) {
    double computation_start_time = ros::Time::now().toSec();

    ocp_state x0 = x0_full.head(6);
    initial_conditions(x0);

//    warmStart();
    double eps = 0.01;
    ocp_state lbx_f, ubx_f;
    ocp().get_state_bounds(lbx_f, ubx_f);
    if(ocp().minimal_time){
        lbx_f = ocp().xs.array()-eps;
        ubx_f = ocp().xs.array()+eps;
    }
    final_state_bounds(lbx_f, ubx_f);

    double time_now = ros::Time::now().toSec();

    MPC::solve();

    last_computation_time = (ros::Time::now().toSec() - time_now) * 1000;
}