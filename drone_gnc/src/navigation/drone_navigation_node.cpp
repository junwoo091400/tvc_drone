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
        DroneEKF::sensor_data new_data;
        new_data.segment(0, 3) << sensor->IMU_gyro.x, sensor->IMU_gyro.y, sensor->IMU_gyro.z;

        kalman.predictStep();
        kalman.updateStep(new_data);
        publishDroneState();
    }

    DroneNavigationNode(ros::NodeHandle &nh) {
        // init publishers and subscribers
        initTopics(nh);

        // init EKF
        kalman.initEKF(nh);

        // Initialize fsm
        current_fsm.time_now = 0;
        current_fsm.state_machine = "Idle";
    }

    void initTopics(ros::NodeHandle &nh) {
        nh.param("drone_dome", DRONE_DOME, false);

        // Create filtered rocket state publisher
        kalman_pub = nh.advertise<drone_gnc::DroneState>("/drone_state", 10);

        // Subscribe to time_keeper for fsm and time
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 100, &DroneNavigationNode::fsmCallback, this);

        // Subscribe to time_keeper for fsm and time
        control_sub = nh.subscribe("/drone_control", 100, &DroneEKF::updateCurrentControl, &kalman);

        sensor_sub = nh.subscribe("/sensors", 100,  &DroneNavigationNode::sensorCallback, this);
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
//    ros::Timer control_thread = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent &) {
//        droneNavigationNode.kalmanStep();
//        droneNavigationNode.publishDroneState();
//    });

    // Automatic callback of service and publisher from here
    ros::spin();
}