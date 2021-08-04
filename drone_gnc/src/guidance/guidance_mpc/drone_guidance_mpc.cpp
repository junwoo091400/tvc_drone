#include "drone_guidance_mpc.h"

DroneGuidanceMPC::DroneGuidanceMPC(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : solution_time(0), drone(drone_ptr) {
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
    if(ocp().minimal_time){
        set_time_limits(0, 1);
    }
    else{
        set_time_limits(0, horizon_length);
    }

    // Setup constraints
    ocp_state lbx, ubx; ocp_control lbu, ubu;
    ocp().get_control_bounds(lbu, ubu);
    ocp().get_state_bounds(lbx, ubx);

    state_bounds(lbx, ubx);
    control_bounds(lbu, ubu);

    //minimal time setup
    parameter_t lbp; parameter_t ubp;
    if(ocp().minimal_time){
        lbp << 0.0;
        ubp << horizon_length;
    }
    else{
        lbp << 0.9;
        ubp << 1.1;
    }
    parameters_bounds(lbp, ubp);


    std::vector<double> initial_target_apogee;
    nh.getParam("target_apogee", initial_target_apogee);
    double z_target = initial_target_apogee.at(2);

    // Initial guess
    parameter_t p0;
    ocp_state x0;
    x0 << 0, 0, 0,
            0, 0, 0;
    Eigen::MatrixXd traj_state_guess = x0.replicate(1, ocp().NUM_NODES);

    if(ocp().minimal_time){
        double max_acc = drone->getThrust(drone->maxPropellerSpeed)/drone->dry_mass - 9.81;
        double horizon_length_guess = sqrt(2*z_target/max_acc); //z = 1/2 a t^2 -> t = sqrt(2z/a)
        ROS_ERROR_STREAM("horizon_length_guess" << horizon_length_guess);
        p0 << horizon_length_guess;

        //z = 1/2 a t^2
        traj_state_guess.row(2) = 0.5*max_acc*time_grid.cwiseProduct(time_grid)*horizon_length_guess*horizon_length_guess;
        //dz = a t
        traj_state_guess.row(5) = max_acc*time_grid*horizon_length_guess;
        ROS_ERROR_STREAM("z guess" << traj_state_guess.row(2));
        ROS_ERROR_STREAM("dz guess" << traj_state_guess.row(5));
    }
    else{
        p0 << 1.0;
    }
    p_guess(p0);

    ocp_control u0;
    u0 << 0, 0, drone->maxPropellerSpeed;
    //TODO scaling
    u_guess(u0.replicate(ocp().NUM_NODES, 1));

    //TODO scaling
    traj_state_guess.resize(varx_size, 1);
    x_guess(traj_state_guess);

    //TODO
//    ocp_state lbx_f;
//    lbx_f << -1, -1, 18, -1, -1, -1;
//    ocp_state ubx_f;
//    ubx_f << 1, 1, 22, 1, 1, 1;
//    final_state_bounds(lbx_f, ubx_f);

    ocp_control target_control;
    target_control << 0, 0, drone->getHoverSpeedAverage();
    ocp_state target_state;
    x0 << 0, 0, 0,
            0, 0, 0;
    setTarget(target_state, target_control);

    init_time = ros::Time::now().toSec();
    fixed_computation_time = mpc_period*0.93;
}


void DroneGuidanceMPC::setTarget(ocp_state &target_state, ocp_control &target_control) {
    ocp().xs << target_state.cwiseProduct(ocp().x_scaling_vec);
    ocp().us << target_control.cwiseProduct(ocp().u_scaling_vec);
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

Drone::state DroneGuidanceMPC::solution_x_at(const double t){
    Drone::state sol;
    sol << MPC::solution_x_at(t).cwiseProduct(ocp().x_unscaling_vec),
            0, 0, 0, 1,
            0, 0, 0;
    return sol;
}

Drone::control DroneGuidanceMPC::solution_u_at(const double t){
    Drone::control sol;
    sol << MPC::solution_u_at(t).cwiseProduct(ocp().u_unscaling_vec), 0;
    return sol;
}

Drone::state DroneGuidanceMPC::solution_x_at(const int t){
    Drone::state sol;
    sol << MPC::solution_x_at(t).cwiseProduct(ocp().x_unscaling_vec),
            0, 0, 0, 1,
            0, 0, 0;
    return sol;
}

Drone::control DroneGuidanceMPC::solution_u_at(const int t){
    Drone::control sol;
    sol << MPC::solution_u_at(t).cwiseProduct(ocp().u_unscaling_vec), 0;
    return sol;
}

double DroneGuidanceMPC::node_time(int i){
    return time_grid(i);
}

void DroneGuidanceMPC::solve(Drone::state &x0_full) {
    double computation_start_time = ros::Time::now().toSec();

    ocp_state x0 = x0_full.head(6);
    initial_conditions(x0);

//    warmStart();

//    double eps = 1;
//    ocp_state lbx_f, ubx_f;
//    ocp().get_state_bounds(lbx_f, ubx_f);
//
//    if(ocp().minimal_time){
//        lbx_f.segment(0, 3) = ocp().xs.segment(0, 3).array()-eps;
//        ubx_f.segment(0, 3) = ocp().xs.segment(0, 3).array()+eps;
//    }
//    final_state_bounds(lbx_f, ubx_f);
//    ROS_ERROR_STREAM("lower bound" << lbx_f);
//    ROS_ERROR_STREAM("upper boundinal" << ubx_f);


    double time_now = ros::Time::now().toSec();

    MPC::solve();

    last_computation_time = (ros::Time::now().toSec()-time_now)*1000;

    time_now = ros::Time::now().toSec();

    solution_time = ros::Time::now().toSec();
}