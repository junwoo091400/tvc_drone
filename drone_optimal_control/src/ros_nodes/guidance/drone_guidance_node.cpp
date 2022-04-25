/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "drone_guidance_node.h"

DroneGuidanceNode::DroneGuidanceNode(ros::NodeHandle &nh, Drone *drone) :
        drone(drone), drone_guidance(drone, (guidance_settings = loadGuidanceSettings(nh), guidance_settings)) {
    // Initialize fsm
    current_fsm.state_machine = rocket_utils::FSM::IDLE;

    if (nh.getParam("mpc/descent_trigger_time", descent_trigger_time)) {
    } else {
        ROS_ERROR("Failed to get Guidance node parameter");
    }

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
    bool control_track_guidance, use_simulation_state;
    nh.param<bool>("/control/track_guidance", control_track_guidance, false);
    nh.param<bool>("/control/use_simulation_state", use_simulation_state, true);
    if (control_track_guidance) {
        target_sub = nh.subscribe("/target_apogee", 1, &DroneGuidanceNode::targetCallback, this);
    }
    if (use_simulation_state) {
        state_sub = nh.subscribe("/rocket_state", 1, &DroneGuidanceNode::simulationStateCallback, this);
    } else {
        state_sub = nh.subscribe("/drone_state", 1, &DroneGuidanceNode::stateCallback, this);
    }

    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneGuidanceNode::fsmCallback, this);

    // Publishers
    horizon_viz_pub = nh.advertise<rocket_utils::Trajectory>("/target_trajectory", 10);

    // Debug
    sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
    qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
    horizon_pub = nh.advertise<drone_optimal_control::DroneTrajectory>("horizon", 10);
    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
}

void DroneGuidanceNode::run() {
    time_compute_start = ros::Time::now();

    if (received_state && current_fsm.state_machine != rocket_utils::FSM::STOP) {
        x0
                << current_state.state.pose.position.x, current_state.state.pose.position.y, current_state.state.pose.position.z,
                current_state.state.twist.linear.x, current_state.state.twist.linear.y, current_state.state.twist.linear.z,
                current_state.state.pose.orientation.x, current_state.state.pose.orientation.y, current_state.state.pose.orientation.z, current_state.state.pose.orientation.w,
                current_state.state.twist.angular.x, current_state.state.twist.angular.y, current_state.state.twist.angular.z;

        double time_until_apogee = drone_guidance.solution_p()(0);
        if (current_fsm.state_machine == rocket_utils::FSM::DESCENT || time_until_apogee < descent_trigger_time) {
            startDescent();
        } else {
            double time_now = ros::Time::now().toSec();
            computeTrajectory();
            last_computation_time = (ros::Time::now().toSec() - time_now) * 1000;

            if (isnan(drone_guidance.solution_x_at(0)(0)) || abs(time_until_apogee) > 1000 ||
                isnan(time_until_apogee)) {
                ROS_ERROR_STREAM("Guidance MPC computation failed");
                drone_guidance.initGuess(x0, target_state);
            } else {
                publishTrajectory();
            }
        }
    }

}

void DroneGuidanceNode::startDescent() {
    if (started_descent) return;
    started_descent = true;
    drone_guidance.setTargetState(drone_guidance.target_land);
    drone_guidance.warmStartDescent(x0);
    drone_guidance.setDescentConstraints();
    computeTrajectory();
    publishTrajectory();
}

void DroneGuidanceNode::fsmCallback(const rocket_utils::FSM::ConstPtr &fsm) {
    current_fsm = *fsm;
}

void DroneGuidanceNode::simulationStateCallback(const rocket_utils::State::ConstPtr &rocket_state) {
    current_state.state = *rocket_state;
    current_state.thrust_scaling = 1;
    current_state.torque_scaling = 1;
    received_state = true;
}

void DroneGuidanceNode::stateCallback(const drone_optimal_control::DroneExtendedState::ConstPtr &rocket_state) {
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

    drone_guidance.setTargetState(new_target_state);

    // recompute a new guess if sudden target change
    if ((target_state.head(3) - new_target_state.head(3)).norm() > 1) {
        x0
                << current_state.state.pose.position.x, current_state.state.pose.position.y, current_state.state.pose.position.z,
                current_state.state.twist.linear.x, current_state.state.twist.linear.y, current_state.state.twist.linear.z,
                current_state.state.pose.orientation.x, current_state.state.pose.orientation.y, current_state.state.pose.orientation.z, current_state.state.pose.orientation.w,
                current_state.state.twist.angular.x, current_state.state.twist.angular.y, current_state.state.twist.angular.z;
        drone_guidance.initGuess(x0, target_state);
    }
    target_state = new_target_state;
}

void DroneGuidanceNode::computeTrajectory() {
    drone->setParams(current_state.thrust_scaling,
                     current_state.torque_scaling,
                     0, 0, 0,
                     0, 0, 0);

    drone_guidance.solve(x0);
}

void DroneGuidanceNode::publishTrajectory() {
    // Send optimal trajectory computed by control. Send only position for now
    rocket_utils::Trajectory trajectory_msg;
    drone_optimal_control::DroneTrajectory horizon_msg;

    for (int i = 0; i < DroneGuidance::num_nodes; i++) {
        Drone::state state_val = drone_guidance.solution_x_at(i);

        rocket_utils::Waypoint point;
        point.time = drone_guidance.node_time(i);
        point.position.x = state_val(0);
        point.position.y = state_val(1);
        point.position.z = state_val(2);
        trajectory_msg.trajectory.push_back(point);

        rocket_utils::State state_msg;
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

        Drone::control control_val = drone_guidance.solution_u_at(i);
        //TODO
        rocket_utils::GimbalControl gimbal_control_msg{};
        rocket_utils::ControlMomentGyro roll_control_msg{};

        drone_optimal_control::DroneWaypointStamped state_msg_stamped;
        state_msg_stamped.state.state = state_msg;
        state_msg_stamped.gimbal_control = gimbal_control_msg;
        state_msg_stamped.roll_control = roll_control_msg;
        state_msg_stamped.header.stamp = time_compute_start + ros::Duration(drone_guidance.node_time(i));
        state_msg_stamped.header.frame_id = ' ';

        horizon_msg.trajectory.push_back(state_msg_stamped);
    }
    horizon_msg.num_node = DroneGuidance::num_nodes;
    horizon_msg.num_segment = NUM_SEG_G;

    horizon_viz_pub.publish(trajectory_msg);
    horizon_msg.header.stamp = time_compute_start;
    horizon_pub.publish(horizon_msg);
}


void DroneGuidanceNode::publishDebugInfo() {
    std_msgs::Int32 msg1;
    msg1.data = drone_guidance.info().iter;
    sqp_iter_pub.publish(msg1);
    std_msgs::Int32 msg2;
    msg2.data = drone_guidance.info().qp_solver_iter;
    qp_iter_pub.publish(msg2);
    std_msgs::Float64 msg3;
    msg3.data = last_computation_time;
    computation_time_pub.publish(msg3);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    DroneProps<double> drone_props = loadDroneProps(nh);
    Drone drone(drone_props);

    DroneGuidanceNode droneGuidanceNode(nh, &drone);

    while (ros::ok()) {
        ros::spinOnce();
        droneGuidanceNode.run();
    }

    return 0;
}
