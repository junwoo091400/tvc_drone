#include "drone_guidance_node.h"

DroneGuidanceNode::DroneGuidanceNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone_mpc(nh, drone_ptr),
                                                                                              drone(drone_ptr) {
    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    nh.getParam("mpc/mpc_period", period);

    std::vector<double> initial_target_apogee;
    nh.getParam("target_apogee", initial_target_apogee);

    target_state << initial_target_apogee.at(0), initial_target_apogee.at(1), initial_target_apogee.at(2),
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    initTopics(nh);
}


void DroneGuidanceNode::initTopics(ros::NodeHandle &nh) {
    // Subscribers
    rocket_state_sub = nh.subscribe("/simu_drone_state", 1, &DroneGuidanceNode::stateCallback, this);
    // target_sub = nh.subscribe("/target_apogee", 1, &DroneGuidanceNode::targetCallback, this);
    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneGuidanceNode::fsmCallback, this);

    // Publishers
    horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/target_trajectory", 10);

    // Debug
    sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
    qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
    horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("horizon", 10);
    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
}

void DroneGuidanceNode::run() {
    double loop_start_time = ros::Time::now().toSec();
    if (received_state) {
        computeTrajectory();
        double p_sol = drone_mpc.solution_p()(0);
        if (isnan(drone_mpc.solution_x_at(0)(0)) || abs(p_sol) > 1000 || isnan(p_sol)) {
            ROS_INFO_STREAM("Guidance MPC computation failed");
            drone_mpc.initGuess(x0, target_state);
        }
        else{
            publishTrajectory();
        }
        publishDebugInfo();
    }
}

void DroneGuidanceNode::fsmCallback(const drone_gnc::FSM::ConstPtr &fsm) {
    current_fsm = *fsm;
}

void DroneGuidanceNode::stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
    current_state = *rocket_state;
    received_state = true;
}

void DroneGuidanceNode::targetCallback(const geometry_msgs::Vector3 &target) {
    Drone::state new_target_state;
    new_target_state << target.x, target.y, target.z,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    target_control << 0, 0, drone->getHoverSpeedAverage(), 0;

    drone_mpc.setTarget(new_target_state, target_control);

    // recompute a new guess if sudden target change
    if((target_state.head(3) - new_target_state.head(3)).norm() > 1){
        x0 << current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
                current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
                current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
                current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;
        drone_mpc.initGuess(x0, target_state);
    }
    target_state = new_target_state;
}

void DroneGuidanceNode::computeTrajectory() {

    Drone::state x0;
    x0 << current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
            current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
            current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
            current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

    drone_mpc.drone->setParams(current_state.thrust_scaling,
                               current_state.torque_scaling,
                               current_state.servo1_offset, current_state.servo2_offset,
                               0, 0, 0,
                               0, 0, 0);

    drone_mpc.solve(x0);
}

void DroneGuidanceNode::publishTrajectory() {
    // Send optimal trajectory computed by control. Send only position for now
    drone_gnc::Trajectory trajectory_msg;
    drone_gnc::DroneTrajectory horizon_msg;

    double traj_length = drone_mpc.solution_p()(0);
    time_compute_start = ros::Time::now();

    for (int i = 0; i < drone_mpc.ocp().NUM_NODES; i++) {
        Drone::state state_val = drone_mpc.solution_x_at(i);

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
        state_msg_stamped.header.stamp = time_compute_start + ros::Duration(drone_mpc.node_time(i) * traj_length);
        state_msg_stamped.header.frame_id = ' ';

        horizon_msg.trajectory.push_back(state_msg_stamped);
    }
    horizon_viz_pub.publish(trajectory_msg);
    horizon_msg.header.stamp = time_compute_start;
    horizon_pub.publish(horizon_msg);
}


void DroneGuidanceNode::publishDebugInfo() {
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
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    std::shared_ptr<Drone> drone = make_shared<Drone>(nh);
    DroneGuidanceNode droneGuidanceNode(nh, drone);

//    // Thread to compute control. Duration defines interval time in seconds
    while (ros::ok()){
        ros::spinOnce();
        droneGuidanceNode.run();
    }
//    ros::Timer control_thread = nh.createTimer(ros::Duration(droneGuidanceNode.period), [&](const ros::TimerEvent &) {
//        droneGuidanceNode.run();
//    });
//    ros::spin();
    return 0;
}
