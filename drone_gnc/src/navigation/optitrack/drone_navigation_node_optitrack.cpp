#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"

#include "geometry_msgs/PoseStamped.h"

#include <std_msgs/Float64.h>

#include "drone_EKF_optitrack.hpp"

class DroneNavigationNodeOptitrack {
private:
    DroneEKFOptitrack kalman;

    drone_gnc::FSM current_fsm;
    drone_gnc::DroneControl current_control;
    drone_gnc::DroneControl previous_control;
    geometry_msgs::PoseStamped optitrack_pose;
    bool received_optitrack = false;
    bool initialized_optitrack = false;
    Eigen::Quaterniond initial_optitrack_orientation;
    Eigen::Vector3d initial_optitrack_position;
    double last_predict_time;
    double last_computation_time = 0;

    ros::Publisher kalman_pub;
    ros::Publisher computation_time_pub;
    ros::Subscriber fsm_sub;
    ros::Subscriber control_sub;
    ros::Subscriber sensor_sub;
public:
    double period;

    DroneNavigationNodeOptitrack(ros::NodeHandle &nh) : kalman(nh) {
        // init publishers and subscribers
        initTopics(nh);

        // Initialize fsm
        current_fsm.time_now = 0;
        current_fsm.state_machine = drone_gnc::FSM::IDLE;

        nh.getParam("period", period);

        //TODO
        initial_optitrack_orientation.setIdentity();
        initial_optitrack_position << 0, 0, 0;

        last_predict_time = ros::Time::now().toSec();
    }

    void initTopics(ros::NodeHandle &nh) {
        // Create filtered rocket state publisher
        kalman_pub = nh.advertise<drone_gnc::DroneState>("/drone_state", 1);

        // Subscribe to time_keeper for fsm and time
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneNavigationNodeOptitrack::fsmCallback, this);

        // Subscribe to time_keeper for fsm and time
        control_sub = nh.subscribe("/drone_control", 1, &DroneNavigationNodeOptitrack::controlCallback, this);

        computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);

        sensor_sub = nh.subscribe("/optitrack_client/Drone/optitrack_pose", 1, &DroneNavigationNodeOptitrack::optitrackCallback,
                                  this);
    }

    void kalmanStep() {
        if (received_optitrack) {
            if (!initialized_optitrack) {
                initial_optitrack_orientation = Eigen::Quaterniond(optitrack_pose.pose.orientation.w,
                                                                   optitrack_pose.pose.orientation.x,
                                                                   optitrack_pose.pose.orientation.y,
                                                                   optitrack_pose.pose.orientation.z);
                initial_optitrack_position = Eigen::Vector3d(optitrack_pose.pose.position.x,
                                                             optitrack_pose.pose.position.y,
                                                             optitrack_pose.pose.position.z);
                initialized_optitrack = true;
            }

            double compute_time_start = ros::Time::now().toSec();
            Eigen::Quaterniond raw_orientation(optitrack_pose.pose.orientation.w, optitrack_pose.pose.orientation.x,
                                               optitrack_pose.pose.orientation.y, optitrack_pose.pose.orientation.z);
            Eigen::Vector3d raw_position(optitrack_pose.pose.position.x, optitrack_pose.pose.position.y,
                                         optitrack_pose.pose.position.z);
            Eigen::Quaterniond orientation = initial_optitrack_orientation.inverse() * raw_orientation;
            Eigen::Vector3d position = raw_position - initial_optitrack_position;

            DroneEKFOptitrack::sensor_data new_data;
            new_data.segment(0, 3) = position;
            new_data.segment(3, 4) = orientation.coeffs();

            Drone::control u;
            u << previous_control.servo1, previous_control.servo2, (previous_control.bottom + previous_control.top) / 2,
                    previous_control.top - previous_control.bottom;
            ros::spinOnce();
            previous_control = current_control;

//            double time_now = optitrack_pose.header.stamp.toSec();
            double time_now = ros::Time::now().toSec();
            double dT = time_now - last_predict_time;
            last_predict_time = time_now;

            kalman.predictStep(dT, u);
            kalman.updateStep(new_data);

            publishDroneState();

            last_computation_time = (ros::Time::now().toSec() - compute_time_start) * 1000;
        }
    }

    // Callback function to store last received fsm
    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm) {
        current_fsm.time_now = fsm->time_now;
        current_fsm.state_machine = fsm->state_machine;
    }

    // Callback function to store last received state
    void rocket_stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
        optitrack_pose.pose = rocket_state->pose;
        optitrack_pose.header.stamp = rocket_state->header.stamp;
        received_optitrack = true;
    }

    void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
        optitrack_pose.pose.position.x = -pose->pose.position.x;
        optitrack_pose.pose.position.y = -pose->pose.position.y;
        optitrack_pose.pose.position.z = pose->pose.position.z;
        optitrack_pose.pose.orientation = pose->pose.orientation;
        optitrack_pose.header.stamp = pose->header.stamp;
        received_optitrack = true;
    }


    void controlCallback(const drone_gnc::DroneControl::ConstPtr &control) {
        current_control = *control;
        kalman.received_control = true;
    }

    void publishDroneState() {
        drone_gnc::DroneState kalman_state;

        kalman_state.pose.position.x = kalman.X(0);
        kalman_state.pose.position.y = kalman.X(1);
        kalman_state.pose.position.z = kalman.X(2);

        kalman_state.twist.linear.x = kalman.X(3);
        kalman_state.twist.linear.y = kalman.X(4);
        kalman_state.twist.linear.z = kalman.X(5);

        kalman_state.pose.orientation.x = kalman.X(6);
        kalman_state.pose.orientation.y = kalman.X(7);
        kalman_state.pose.orientation.z = kalman.X(8);
        kalman_state.pose.orientation.w = kalman.X(9);

        kalman_state.twist.angular.x = kalman.X(10);
        kalman_state.twist.angular.y = kalman.X(11);
        kalman_state.twist.angular.z = kalman.X(12);

        kalman_state.thrust_scaling = kalman.X(13);
        kalman_state.torque_scaling = kalman.X(14);
        kalman_state.disturbance_force.x = kalman.X(15);
        kalman_state.disturbance_force.y = kalman.X(16);
        kalman_state.disturbance_force.z = kalman.X(17);
        kalman_state.disturbance_torque.x = kalman.X(18);
        kalman_state.disturbance_torque.y = kalman.X(19);
        kalman_state.disturbance_torque.z = kalman.X(20);

        kalman_state.header.stamp = ros::Time::now();

        kalman_pub.publish(kalman_state);

        std_msgs::Float64 msg3;
        msg3.data = last_computation_time;
        computation_time_pub.publish(msg3);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("navigation");

    DroneNavigationNodeOptitrack droneNavigationNode(nh);

    // Thread to compute kalman. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(droneNavigationNode.period), [&](const ros::TimerEvent &) {
        droneNavigationNode.kalmanStep();
    });

    // Automatic callback of service and publisher from here
    ros::spin();
}