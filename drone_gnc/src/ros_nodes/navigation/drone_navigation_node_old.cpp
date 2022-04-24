/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This source code is subject to the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. If a copy of the GNU General Public License was not distributed
 * with this file, you can obtain one at http://www.gnu.org/licenses/.
 */
#include "drone_navigation_node_old.h"

DroneNavigationNode::DroneNavigationNode(ros::NodeHandle &nh) :
        drone((drone_props = loadDroneProps(nh), drone_props)),
        kalman(&drone, (navigation_settings = loadNavigationSettings(nh), navigation_settings)) {
    // init publishers and subscribers
    initTopics(nh);

    period = navigation_settings.period;

    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = rocket_utils::FSM::IDLE;

    measured_drone_state.setZero();
    measured_drone_state(9) = 1;

    origin.setZero();

    init_time = ros::Time::now().toSec();

    last_predict_time = ros::Time::now().toSec();

}

void DroneNavigationNode::initTopics(ros::NodeHandle &nh) {
    // Create filtered rocket state publisher
    kalman_pub = nh.advertise<drone_gnc::DroneExtendedState>("/drone_state", 1);

    // Subscribe to time_keeper for fsm and time
    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneNavigationNode::fsmCallback, this);

    // Subscribe to time_keeper for fsm and time
    control_sub = nh.subscribe("/drone_control", 1, &DroneNavigationNode::controlCallback, this);

    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);

    if (use_gps) {
        initialized_orientation = true;
        // /!\ the tcpNoDelay option is apparently necessary to avoid delays for large message types such as Odometry
        pixhawk_ekf_sub = nh.subscribe("/mavros/global_position/local", 1,
                                       &DroneNavigationNode::pixhawkEKFCallback, this,
                                       ros::TransportHints().tcpNoDelay());
        pixhawk_twist_body_sub = nh.subscribe("/mavros/local_position/velocity_body", 1,
                                              &DroneNavigationNode::pixhawkTwistBodyCallback, this);
    } else {
        pixhawk_pose_sub = nh.subscribe("/mavros/local_position/pose", 1,
                                        &DroneNavigationNode::pixhawkPoseCallback, this);
//        pixhawk_twist_local_sub = nh.subscribe("/mavros/local_position/velocity_local", 1,
//                                               &DroneNavigationNodePixhawk::pixhawkTwistLocalCallback, this);
        pixhawk_twist_body_sub = nh.subscribe("/mavros/local_position/velocity_body", 1,
                                              &DroneNavigationNode::pixhawkTwistBodyCallback, this);
        optitrack_sub = nh.subscribe("/optitrack_client/Drone/optitrack_pose", 1,
                                     &DroneNavigationNode::optitrackCallback, this);
    }

}

void DroneNavigationNode::kalmanStep() {
    if (received_pixhawk && initialized_orientation) {
        if ((received_optitrack || use_gps) && current_fsm.state_machine == rocket_utils::FSM::IDLE) {
            origin = measured_drone_state.segment(0, 3);
        }

        double compute_time_start = ros::Time::now().toSec();

        DroneEKF::sensor_data new_data = measured_drone_state;
        new_data.segment(0, 3) -= origin;

        Drone::control u;
        u << previous_control.servo1, previous_control.servo2, (previous_control.bottom + previous_control.top) / 2,
                previous_control.top - previous_control.bottom;
        previous_control = current_control;

//            double time_now = optitrack_pose.header.stamp.toSec();
        double time_now = ros::Time::now().toSec();
        double dT = time_now - last_predict_time;
        last_predict_time = time_now;

        kalman.predictStep(dT, u);

        ros::spinOnce();
        if (update_trigger) {
            kalman.updateStep(new_data);
            update_trigger = false;
        }

        publishDroneState();

        last_computation_time = (ros::Time::now().toSec() - compute_time_start) * 1000;
    }
}

// Callback function to store last received fsm
void DroneNavigationNode::fsmCallback(const rocket_utils::FSM::ConstPtr &fsm) {
    current_fsm.time_now = fsm->time_now;
    current_fsm.state_machine = fsm->state_machine;
}

// Callback function to store last received state
void DroneNavigationNode::pixhawkEKFCallback(const nav_msgs::Odometry::ConstPtr &state) {
    measured_drone_state.head(10)
            << state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z,
            state->twist.twist.linear.x, state->twist.twist.linear.y, -state->twist.twist.linear.z,
            state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z, state->pose.pose.orientation.w;

    received_pixhawk = true;
}

// Callback function to store last received state
void DroneNavigationNode::pixhawkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
    if (!initialized_orientation && (ros::Time::now().toSec() - init_time) > 3) {
        initial_orientation.coeffs()
                << pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w;
        initialized_orientation = true;
    }
    if (initialized_orientation) {
        Quaterniond orientation;
        orientation.coeffs()
                << pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w;

        measured_drone_state.segment(6, 4) = (initial_orientation.inverse() * orientation).coeffs();
    }
//    measured_drone_state.segment(6, 4) = orientation.coeffs();
    received_pixhawk = true;
}

// Callback function to store last received state
void DroneNavigationNode::pixhawkTwistBodyCallback(const geometry_msgs::TwistStamped::ConstPtr &twist) {
    measured_drone_state.segment(10, 3) << twist->twist.angular.x, twist->twist.angular.y, twist->twist.angular.z;
    update_trigger = true;
}

// Callback function to store last received state
void DroneNavigationNode::pixhawkTwistLocalCallback(const geometry_msgs::TwistStamped::ConstPtr &twist) {
    if (initialized_orientation) {
        Vector3d raw_velocity;
        raw_velocity << twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z;
        Vector3d velocity = initial_orientation.inverse()._transformVector(raw_velocity);
        measured_drone_state.segment(3, 3) = velocity;
    }
}

// Callback function to store last received state
void DroneNavigationNode::optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
    measured_drone_state.head(3) << -pose->pose.position.x, -pose->pose.position.y, pose->pose.position.z;
    received_optitrack = true;
}

void DroneNavigationNode::controlCallback(const drone_gnc::DroneControl::ConstPtr &control) {
    current_control = *control;
    if (!kalman.received_control) {
        kalman.startParamEstimation();
    }
}

void DroneNavigationNode::publishDroneState() {
    drone_gnc::DroneExtendedState kalman_state;

    kalman_state.pose.position.x = kalman.getState(0);
    kalman_state.pose.position.y = kalman.getState(1);
    kalman_state.pose.position.z = kalman.getState(2);

    kalman_state.twist.linear.x = kalman.getState(3);
    kalman_state.twist.linear.y = kalman.getState(4);
    kalman_state.twist.linear.z = kalman.getState(5);

    kalman_state.pose.orientation.x = kalman.getState(6);
    kalman_state.pose.orientation.y = kalman.getState(7);
    kalman_state.pose.orientation.z = kalman.getState(8);
    kalman_state.pose.orientation.w = kalman.getState(9);

    kalman_state.twist.angular.x = kalman.getState(10);
    kalman_state.twist.angular.y = kalman.getState(11);
    kalman_state.twist.angular.z = kalman.getState(12);

    kalman_state.thrust_scaling = kalman.getState(13);
    kalman_state.torque_scaling = kalman.getState(14);
    kalman_state.disturbance_force.x = kalman.getState(15);
    kalman_state.disturbance_force.y = kalman.getState(16);
    kalman_state.disturbance_force.z = kalman.getState(17);
    kalman_state.disturbance_torque.x = kalman.getState(18);
    kalman_state.disturbance_torque.y = kalman.getState(19);
    kalman_state.disturbance_torque.z = kalman.getState(20);

    kalman_state.header.stamp = ros::Time::now();

    kalman_pub.publish(kalman_state);

    std_msgs::Float64 msg3;
    msg3.data = last_computation_time;
    computation_time_pub.publish(msg3);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("navigation");

    DroneNavigationNode droneNavigationNode(nh);

    // Thread to compute kalman. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(droneNavigationNode.period), [&](const ros::TimerEvent &) {
        ros::spinOnce();
        droneNavigationNode.kalmanStep();
    });

    // Automatic callback of service and publisher from here
    ros::spin();
}