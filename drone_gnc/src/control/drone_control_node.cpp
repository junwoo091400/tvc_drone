#include "drone_control_node.h"

DroneControlNode::DroneControlNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone_mpc(nh, drone_ptr),
                                                                                            drone(drone_ptr),
                                                                                            backupController(
                                                                                                    drone_ptr) {
    initTopics(nh);

    std::vector<double> initial_target_apogee;
    nh.getParam("target_apogee", initial_target_apogee);
    target_apogee.x = initial_target_apogee.at(0);
    target_apogee.y = initial_target_apogee.at(1);
    target_apogee.z = initial_target_apogee.at(2);

    //init segment length
//    double guidance_horizon_length;
//    nh.getParam("guidance/mpc/horizon_length", guidance_horizon_length);
    sgm_length = 1.0 / GUIDANCE_NUM_SEG;

    nh.param("/fixed_guidance", fixed_guidance, false);
    nh.param("track_guidance", track_guidance, false);

    nh.getParam("mpc/mpc_period", period);
    nh.getParam("mpc/feedforward_period", feedforward_period);

    int max_ff_index = std::round(period / feedforward_period) - 1;

//    low_level_control_thread = nh.createTimer(
//            ros::Duration(feedforward_period), [&](const ros::TimerEvent &) {
//                if (ff_index < max_ff_index) {
//                    publishFeedforwardControl();
//                    ff_index++;
//                }
//            });


    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";
}

void DroneControlNode::initTopics(ros::NodeHandle &nh) {
    // Subscribers
    rocket_state_sub = nh.subscribe("/drone_state", 1, &DroneControlNode::stateCallback, this);
    target_sub = nh.subscribe("/target_apogee", 1, &DroneControlNode::targetCallback, this);
    target_traj_sub = nh.subscribe("/guidance/horizon", 1, &DroneControlNode::targetTrajectoryCallback, this);
    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneControlNode::fsmCallback, this);

    // Publishers
    horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/mpc_horizon", 10);
    drone_control_pub = nh.advertise<drone_gnc::DroneControl>("/drone_control", 10);

    // Debug
    sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
    qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
    horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("debug/horizon", 10);
    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
}

void DroneControlNode::run() {
    double loop_start_time = ros::Time::now().toSec();
    // State machine ------------------------------------------
    if (received_trajectory || !track_guidance) {

        fetchNewTarget();
        if (!USE_BACKUP_CONTROLLER) {
            computeControl();
        }

        if (current_fsm.state_machine == "Launch") {
            publishFeedforwardControl();
            if(start_time == 0)start_time = ros::Time::now().toSec();
        }

        publishTrajectory();
        saveDebugInfo();
        publishDebugInfo();

//        if (current_fsm.state_machine.compare("Launch") == 0) {
//            low_level_control_thread.stop();
//            publishFeedforwardControl();
//            ff_index = 0;
//            low_level_control_thread.start();
//        }
    }
}

void DroneControlNode::fsmCallback(const drone_gnc::FSM::ConstPtr &fsm) {
    current_fsm = *fsm;
}

// Callback function to store last received state
void DroneControlNode::stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
    const std::lock_guard<std::mutex> lock(state_mutex);
    current_state = *rocket_state;
    received_state = true;
}

// Callback function to store last received state
void DroneControlNode::targetCallback(const geometry_msgs::Vector3 &target) {
    const std::lock_guard<std::mutex> lock(target_mutex);
    target_apogee = target;
}


void DroneControlNode::targetTrajectoryCallback(const drone_gnc::DroneTrajectory::ConstPtr &target) {
    guidance_t0 = target->trajectory.at(0).header.stamp.toSec();
    guidance_tf = target->trajectory.at(GUIDANCE_NUM_NODE - 1).header.stamp.toSec();

    if (!received_trajectory && !fixed_guidance) {
        // init lagrange basis

        double traj_length = guidance_tf - guidance_t0;
        Eigen::Matrix<double, GUIDANCE_POLY_ORDER + 1, 1> time_grid;
        for (int i = 0; i < GUIDANCE_POLY_ORDER + 1; i++) {
            time_grid(i) = (target->trajectory.at(i).header.stamp.toSec() - guidance_t0)/traj_length;
        }
        polympc::LagrangeSpline::compute_lagrange_basis(time_grid, m_basis);
    }
    received_trajectory = true;

    int i = 0;
    for (auto waypoint: target->trajectory) {
        guidanceTrajectory.col(i)
                << waypoint.state.pose.position.x, waypoint.state.pose.position.y, waypoint.state.pose.position.z,
                waypoint.state.twist.linear.x, waypoint.state.twist.linear.y, waypoint.state.twist.linear.z,
                waypoint.state.pose.orientation.x, waypoint.state.pose.orientation.y, waypoint.state.pose.orientation.z, waypoint.state.pose.orientation.w,
                waypoint.state.twist.angular.x, waypoint.state.twist.angular.y, waypoint.state.twist.angular.z;
        i++;
    }
}

Drone::state DroneControlNode::sampleTargetTrajectory(double t) {
    Eigen::Index idx = (Eigen::Index) std::floor(t / sgm_length);
    idx = std::max(Eigen::Index(0),
                   std::min(idx, (Eigen::Index) GUIDANCE_NUM_SEG - 1)); // clip idx to stay within the spline bounds

    Drone::state state = polympc::LagrangeSpline::eval(t - idx * sgm_length,
                                                       guidanceTrajectory.block<Drone::NX, GUIDANCE_POLY_ORDER + 1>(
                                                               0, idx * GUIDANCE_POLY_ORDER), m_basis);
    return state;
}

void DroneControlNode::sampleTargetTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_traj) {
    double time_now = ros::Time::now().toSec();
    //first value is to make startup work, second value is used in general
    double time_delta = std::min(time_now - start_time, time_now - guidance_t0);

    for (int i = 0; i < DroneMPC::num_nodes; i++) {
        double t = (time_delta + drone_mpc.time_grid(i))/(guidance_tf-guidance_t0);

        t = min(t, 1.0);

        Eigen::Index idx = (Eigen::Index) std::floor(t / sgm_length);
        idx = std::max(Eigen::Index(0), std::min(idx, (Eigen::Index) GUIDANCE_NUM_SEG - 1));

        Drone::state state = polympc::LagrangeSpline::eval(t - idx * sgm_length, guidanceTrajectory.block<Drone::NX,
                GUIDANCE_POLY_ORDER + 1>(0, idx * GUIDANCE_POLY_ORDER), m_basis);

        mpc_target_traj.col(i) = state.cwiseProduct(drone_mpc.ocp().x_drone_scaling_vec);
    }
}


void DroneControlNode::sampleTargetTrajectoryLinear(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_traj) {
    double time_now = ros::Time::now().toSec();

    for (int i = 0; i < DroneMPC::num_nodes; i++) {
        double t = time_now + drone_mpc.time_grid(i) - guidance_t0;
        //TODO remove constants
        double idx = t * GUIDANCE_NUM_NODE / (guidance_tf - guidance_t0);

        //linear interpolation
        int l_idx = floor(idx);
        l_idx = min(l_idx, GUIDANCE_NUM_NODE - 1);
        Drone::state l_state = guidanceTrajectory.col(l_idx);

        int u_idx = ceil(idx);
        u_idx = min(u_idx, GUIDANCE_NUM_NODE - 1);
        Drone::state u_state = guidanceTrajectory.col(u_idx);

        Drone::state state = l_state + (u_state - l_state) * (idx - l_idx);
        mpc_target_traj.col(i) = state.cwiseProduct(drone_mpc.ocp().x_drone_scaling_vec);
    }
}


void DroneControlNode::computeControl() {
    time_compute_start = ros::Time::now().toSec();


    Drone::state x0;
    state_mutex.lock();
    x0 << current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
            current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
            current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
            current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;
    state_mutex.unlock();

    drone_mpc.drone->setParams(current_state.thrust_scaling,
                               current_state.torque_scaling,
                               current_state.servo1_offset, current_state.servo2_offset,
                               current_state.disturbance_force.x, current_state.disturbance_force.y,
                               current_state.disturbance_force.z,
                               current_state.disturbance_torque.x, current_state.disturbance_torque.y,
                               current_state.disturbance_torque.z);


    drone_mpc.solve(x0);

    //TODO
    if (isnan(drone_mpc.solution_x_at(0)(0))) {
        ROS_ERROR("MPC ISSUE\n");
        DroneMPC::ocp_state x0;
        x0 << 0, 0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                0, 0, 0;
        drone_mpc.x_guess(x0.cwiseProduct(drone_mpc.ocp().x_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));
        DroneMPC::ocp_control u0;
        u0 << 0, 0, drone->getHoverSpeedAverage(), 0;
        drone_mpc.u_guess(u0.cwiseProduct(drone_mpc.ocp().u_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));

        DroneMPC::dual_var_t dual;
        dual.setZero();
        drone_mpc.lam_guess(dual);
    }

}

void DroneControlNode::publishFeedforwardControl() {
    drone_gnc::DroneControl drone_control;
    if (USE_BACKUP_CONTROLLER) {
        drone_control = backupController.getControl(current_state, target_apogee);
    } else {
        drone_control = drone_mpc.getControlCurrentTime();
    }

    drone_control_pub.publish(drone_control);
}

void DroneControlNode::publishTrajectory() {

    // Send optimal trajectory computed by control. Send only position for now
    drone_gnc::Trajectory trajectory_msg;
    drone_gnc::DroneTrajectory horizon_msg;
    for (int i = 0; i < drone_mpc.ocp().NUM_NODES; i++) {
        Drone::state state_val = drone_mpc.solution_x_at(i);
//            Drone::state state_val = drone_mpc.ocp().targetTrajectory.col(i);

        drone_gnc::Waypoint point;
        point.time = drone_mpc.node_time(i);
        point.position.x = state_val(0);
        point.position.y = state_val(1);
        point.position.z = state_val(2);
        trajectory_msg.trajectory.push_back(point);

        drone_gnc::DroneState state_msg;
        state_msg.pose.position.x = state_val(0);
        state_msg.pose.position.y = state_val(1);
        state_msg.pose.position.z = state_val(2);

        state_msg.twist.linear.x = state_val(3);
        state_msg.twist.linear.y = state_val(4);
        state_msg.twist.linear.z = state_val(5);

        state_msg.pose.orientation.x = state_val(6);
        state_msg.pose.orientation.y = state_val(7);
        state_msg.pose.orientation.z = state_val(8);
        state_msg.pose.orientation.w = state_val(9);

        state_msg.twist.angular.x = state_val(10);
        state_msg.twist.angular.y = state_val(11);
        state_msg.twist.angular.z = state_val(12);

        Drone::control control_val = drone_mpc.solution_u_at(i);
        drone_gnc::DroneControl control_msg;
        control_msg.servo1 = control_val(0);
        control_msg.servo2 = control_val(1);
        control_msg.bottom = control_val(2) - control_val(3) / 2;
        control_msg.top = control_val(2) + control_val(3) / 2;

        drone_gnc::DroneWaypointStamped state_msg_stamped;
        state_msg_stamped.state = state_msg;
        state_msg_stamped.control = control_msg;
        state_msg_stamped.header.stamp = ros::Time::now() + ros::Duration(drone_mpc.node_time(i));
        state_msg_stamped.header.frame_id = ' ';


        horizon_msg.trajectory.push_back(state_msg_stamped);
    }
    horizon_msg.header.stamp = ros::Time::now();
    horizon_viz_pub.publish(trajectory_msg);
    horizon_pub.publish(horizon_msg);
}


void DroneControlNode::fetchNewTarget() {
    Drone::state target_state;
    Drone::control target_control;

    target_mutex.lock();
    target_state << target_apogee.x, target_apogee.y, target_apogee.z,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    target_mutex.unlock();

    target_control << 0, 0, drone->getHoverSpeedAverage(), 0;
//        }

    drone_mpc.setTarget(target_state, target_control);

    Matrix<double, Drone::NX, DroneMPC::num_nodes> mpc_target_traj;
    if (!track_guidance) {
        for (int i = 0; i < DroneMPC::num_nodes; i++) {
            mpc_target_traj.col(i) = target_state.cwiseProduct(drone_mpc.ocp().x_drone_scaling_vec);
        }
    } else if (fixed_guidance) {
        sampleTargetTrajectoryLinear(mpc_target_traj);
    } else {
        sampleTargetTrajectory(mpc_target_traj);
    }

    //TODO separate
    drone_mpc.ocp().targetTrajectory = mpc_target_traj;
}

void DroneControlNode::saveDebugInfo() {
    double x_error = current_state.pose.position.x - target_apogee.x;
    average_x_error.push_back(x_error * x_error);
    double y_error = current_state.pose.position.y - target_apogee.y;
    average_y_error.push_back(y_error * y_error);
    double z_error = current_state.pose.position.z - target_apogee.z;
    average_z_error.push_back(z_error * z_error);

    average_status.push_back(drone_mpc.info().status.value);
    average_time.push_back(time_compute_start);
}

void DroneControlNode::printDebugInfo() {
    std::cout << "Average time: "
              << (std::accumulate(average_time.begin(), average_time.end(), 0.0)) / average_time.size()
              << "ms | Average error (x, y, z): "
              << (std::accumulate(average_x_error.begin(), average_x_error.end(), 0.0)) / average_z_error.size()
              << ", "
              << (std::accumulate(average_y_error.begin(), average_y_error.end(), 0.0)) / average_z_error.size()
              << ", "
              << (std::accumulate(average_z_error.begin(), average_z_error.end(), 0.0)) / average_z_error.size()
              << "\n";
}

void DroneControlNode::publishDebugInfo() {
    std_msgs::Int32 msg1;
    msg1.data = drone_mpc.info().iter;
    sqp_iter_pub.publish(msg1);
    std_msgs::Int32 msg2;
    msg2.data = drone_mpc.info().qp_solver_iter;
    qp_iter_pub.publish(msg2);
    std_msgs::Float64 msg3;
    msg3.data = drone_mpc.last_computation_time;
    computation_time_pub.publish(msg3);
}


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("control");

    std::shared_ptr<Drone> drone = make_shared<Drone>(nh);

    DroneControlNode droneControlNode(nh, drone);

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(droneControlNode.period), [&](const ros::TimerEvent &) {
        droneControlNode.run();
    });

    // Start spinner on a different thread to allow spline evaluation while a new solution is computed
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
}
