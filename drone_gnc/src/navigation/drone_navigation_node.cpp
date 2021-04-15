#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/Sensor.h"

#include "geometry_msgs/PoseStamped.h"

#include <time.h>

#include "drone_EKF.h"

bool DRONE_DOME;

// Global variable with last received fsm
drone_gnc::FSM current_fsm;

// Global variable with last received control
drone_gnc::DroneControl current_control;
// Callback function to store last received fsm
void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm) {
    current_fsm.time_now = fsm->time_now;
    current_fsm.state_machine = fsm->state_machine;
}

// Callback function to store last received sensor data
void sensorCallback(const drone_gnc::Sensor::ConstPtr &sensor) {
}

geometry_msgs::Pose optitrack_pose;
bool received_optitrack = false;

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

int main(int argc, char **argv) {

    DroneEKF kalman;

    // Init ROS time keeper node
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("navigation");

    nh.param("drone_dome", DRONE_DOME, false);

    // Create filtered rocket state publisher
    ros::Publisher kalman_pub = nh.advertise<drone_gnc::DroneState>("/drone_state", 10);

    // Subscribe to time_keeper for fsm and time
    ros::Subscriber fsm_sub = nh.subscribe("/gnc_fsm_pub", 100, fsmCallback);


    // Subscribe to time_keeper for fsm and time
    ros::Subscriber control_sub = nh.subscribe("/drone_control", 100, &DroneEKF::update_current_control, &kalman);

    ros::Subscriber sensor_sub;
    if (DRONE_DOME) {
        // sensor_sub = nh.subscribe("/simu_drone_state", 100, rocket_stateCallback);
       sensor_sub = nh.subscribe("/optitrack_client/Kite/optitrack_pose", 100, optitrackCallback);
    } else {
        sensor_sub = nh.subscribe("/sensors", 100, sensorCallback);
    }

    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    // init EKF
    kalman.init_EKF(nh);

    Eigen::Quaterniond initial_optitrack_orientation(1, 0, 0, 0);
    Eigen::Vector3d initial_optitrack_position(0, 0, 0);
    bool initialized_optitrack = false;

    kalman.last_predict_time = ros::Time::now().toSec();
    // Thread to compute kalman. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent &) {
        //double time_now = ros::Time::now().toSec();
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

            kalman.predict_step();
            kalman.optitrack_update_step(new_data);
        }

        // Parse kalman state and publish it on the /kalman_pub topic
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

        kalman_pub.publish(kalman_state);

        //std::cout << 1000*(ros::Time::now().toSec()-time_now) << "\n";

    });

    // Automatic callback of service and publisher from here
    ros::spin();

}