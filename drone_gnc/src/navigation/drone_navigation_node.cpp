#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/Sensor.h"

#include "geometry_msgs/PoseStamped.h"

#include <time.h>

#include "drone_EKF.h"

bool DRONE_DOME;

class DroneNavigationNode {
public:
    // Callback function to store last received fsm
    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm) {
        current_fsm.time_now = fsm->time_now;
        current_fsm.state_machine = fsm->state_machine;
    }

    // Callback function to store last received sensor data
    void sensorCallback(const drone_gnc::Sensor::ConstPtr &sensor) {
    }

    // Callback function to store last received state
    void rocket_stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
        optitrack_pose = rocket_state->pose;
        received_optitrack = true;
    }

    void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
        optitrack_pose.position.x = -pose->pose.position.x;
        optitrack_pose.position.y = -pose->pose.position.y;
        optitrack_pose.position.z = pose->pose.position.z;
        optitrack_pose.orientation = pose->pose.orientation;
        received_optitrack = true;
    }

    DroneNavigationNode(ros::NodeHandle &nh) {
        // init publishers and subscribers
        initTopics(nh);

        // init EKF
        kalman.initEKF(nh);

        // Initialize fsm
        current_fsm.time_now = 0;
        current_fsm.state_machine = "Idle";

        //TODO
        initial_optitrack_orientation.setIdentity();
        initial_optitrack_position << 0, 0, 0;
    }

    void initTopics(ros::NodeHandle &nh) {
        nh.param("drone_dome", DRONE_DOME, false);

        // Create filtered rocket state publisher
        kalman_pub = nh.advertise<drone_gnc::DroneState>("/drone_state", 10);

        // Subscribe to time_keeper for fsm and time
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 100, &DroneNavigationNode::fsmCallback, this);

        // Subscribe to time_keeper for fsm and time
        control_sub = nh.subscribe("/drone_control", 100, &DroneEKF::updateCurrentControl, &kalman);

        sensor_sub;
        if (DRONE_DOME) {
            sensor_sub = nh.subscribe("/simu_drone_state", 100, &DroneNavigationNode::rocket_stateCallback, this);
//        sensor_sub = nh.subscribe("/optitrack_client/Kite/optitrack_pose", 100, &DroneNavigationNode::optitrackCallback, this);
        } else {
            sensor_sub = nh.subscribe("/sensors", 100,  &DroneNavigationNode::sensorCallback, this);
        }
    }

    void kalmanStep() {
        if (received_optitrack) {
            if (!initialized_optitrack) {
                initial_optitrack_orientation = Eigen::Quaterniond(optitrack_pose.orientation.w,
                                                                   optitrack_pose.orientation.x,
                                                                   optitrack_pose.orientation.y,
                                                                   optitrack_pose.orientation.z);
                initial_optitrack_position = Eigen::Vector3d(optitrack_pose.position.x, optitrack_pose.position.y,
                                                             optitrack_pose.position.z);
                initialized_optitrack = true;
            }

            Eigen::Quaterniond raw_orientation(optitrack_pose.orientation.w, optitrack_pose.orientation.x,
                                               optitrack_pose.orientation.y, optitrack_pose.orientation.z);
            Eigen::Vector3d raw_position(optitrack_pose.position.x, optitrack_pose.position.y,
                                         optitrack_pose.position.z);
            Eigen::Quaterniond orientation = initial_optitrack_orientation.inverse() * raw_orientation;
            Eigen::Vector3d position = raw_position - initial_optitrack_position;

            Matrix<double, 7, 1> new_data;
            new_data.segment(0, 3) = position;
            new_data.segment(3, 4) = orientation.coeffs();

            kalman.predictStep();
            kalman.optitrackUpdateStep(new_data);
        }
    }

    void publishDroneState(){
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

        kalman_state.disturbance_torque.x = kalman.X(14);
        kalman_state.disturbance_torque.y = kalman.X(15);
        kalman_state.disturbance_torque.z = kalman.X(16);

        kalman_pub.publish(kalman_state);
    }

private:
    DroneEKF kalman;

    drone_gnc::FSM current_fsm;
    drone_gnc::DroneControl current_control;
    geometry_msgs::Pose optitrack_pose;
    bool received_optitrack = false;
    bool initialized_optitrack = false;
    Eigen::Quaterniond initial_optitrack_orientation;
    Eigen::Vector3d initial_optitrack_position;

    ros::Publisher kalman_pub;
    ros::Subscriber fsm_sub;
    ros::Subscriber control_sub;
    ros::Subscriber sensor_sub;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("navigation");

    DroneNavigationNode droneNavigationNode(nh);

    // Thread to compute kalman. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent &) {
        droneNavigationNode.kalmanStep();
        droneNavigationNode.publishDroneState();
    });

    // Automatic callback of service and publisher from here
    ros::spin();
}