#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/Sensor.h"
#include "drone_gnc/KalmanSimu.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <time.h>

#include "pixhawk/drone_EKF_pixhawk.hpp"

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <vector> // for vectors

using namespace std;
using namespace Eigen;

DroneEKFPixhawk *kalman;

double period;
std::string log_file_path;

bool kalmanSimu(drone_gnc::KalmanSimu::Request &req, drone_gnc::KalmanSimu::Response &res) {
    bool initialized_origin = false;

    kalman->estimate_params = req.estimate_params;

    DroneEKFPixhawk::state Q(Map<DroneEKFPixhawk::state>(req.Q.data()));
    DroneEKFPixhawk::sensor_data R(Map<DroneEKFPixhawk::sensor_data>(req.R.data()));
    kalman->setQdiagonal(Q);
    kalman->setRdiagonal(R);
    kalman->reset();

    drone_gnc::DroneTrajectory state_history;
    rosbag::Bag bag;
    bag.open(ros::package::getPath("drone_utils") + "/" + log_file_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/optitrack_client/Drone/optitrack_pose"));
    topics.push_back(std::string("/mavros/local_position/pose"));
    topics.push_back(std::string("/mavros/local_position/velocity_body"));
    topics.push_back(std::string("/drone_control"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    drone_gnc::DroneControl current_control, previous_control;
    Vector3d origin;
    double last_predict_time;

    geometry_msgs::PoseStamped optitrack_pose;
    geometry_msgs::PoseStamped pixhawk_pose;
    geometry_msgs::TwistStamped pixhawk_twist;

    double t0 = 0;
    double current_time = 0;
    for (rosbag::MessageInstance const m: view) {

        drone_gnc::DroneControl::ConstPtr current_control_ptr = m.instantiate<drone_gnc::DroneControl>();
        if (current_control_ptr != NULL) {
            current_control = *current_control_ptr;
            last_predict_time = current_control.header.stamp.toSec();
            kalman->received_control = true;
        }

        if (m.getTopic() == "/optitrack_client/Drone/optitrack_pose") {
            optitrack_pose = *m.instantiate<geometry_msgs::PoseStamped>();
        }
        if (m.getTopic() == "/mavros/local_position/pose") {
            pixhawk_pose = *m.instantiate<geometry_msgs::PoseStamped>();
            current_time = pixhawk_pose.header.stamp.toSec();
            if (t0 == 0) {
                t0 = current_time;
            }
        }
        if (m.getTopic() == "/mavros/local_position/velocity_local") {
            pixhawk_twist = *m.instantiate<geometry_msgs::TwistStamped>();
        }

        if (!initialized_origin) {
            origin << pixhawk_pose.pose.orientation.x, pixhawk_pose.pose.orientation.y, pixhawk_pose.pose.orientation.z;
            initialized_origin = true;
        }

//        ROS_ERROR_STREAM(current_time - t0);
        if (current_time - last_predict_time > period) {
            DroneEKFPixhawk::sensor_data new_data;
            new_data
                    << optitrack_pose.pose.position.x, optitrack_pose.pose.position.y, optitrack_pose.pose.position.z,
                    pixhawk_twist.twist.linear.x, pixhawk_twist.twist.linear.y, pixhawk_twist.twist.linear.z,
                    pixhawk_pose.pose.orientation.x, pixhawk_pose.pose.orientation.y, pixhawk_pose.pose.orientation.z, pixhawk_pose.pose.orientation.w,
                    pixhawk_twist.twist.angular.x, pixhawk_twist.twist.angular.y, pixhawk_twist.twist.angular.z;
            new_data.segment(0, 3) -= origin;

            Drone::control u;
            u << previous_control.servo1, previous_control.servo2,
                    (previous_control.bottom + previous_control.top) / 2,
                    previous_control.top - previous_control.bottom;
            previous_control = current_control;

            kalman->predictStep(current_time - last_predict_time, u);
            kalman->updateStep(new_data);

            last_predict_time = current_time;

            drone_gnc::DroneState kalman_state;

            kalman_state.pose.position.x = kalman->X(0);
            kalman_state.pose.position.y = kalman->X(1);
            kalman_state.pose.position.z = kalman->X(2);

            kalman_state.twist.linear.x = kalman->X(3);
            kalman_state.twist.linear.y = kalman->X(4);
            kalman_state.twist.linear.z = kalman->X(5);

            kalman_state.pose.orientation.x = kalman->X(6);
            kalman_state.pose.orientation.y = kalman->X(7);
            kalman_state.pose.orientation.z = kalman->X(8);
            kalman_state.pose.orientation.w = kalman->X(9);

            kalman_state.twist.angular.x = kalman->X(10);
            kalman_state.twist.angular.y = kalman->X(11);
            kalman_state.twist.angular.z = kalman->X(12);

            kalman_state.thrust_scaling = kalman->X(13);
            kalman_state.torque_scaling = kalman->X(14);
            kalman_state.servo1_offset = kalman->X(15);
            kalman_state.servo2_offset = kalman->X(16);
            kalman_state.disturbance_force.x = kalman->X(17);
            kalman_state.disturbance_force.y = kalman->X(18);
            kalman_state.disturbance_force.z = kalman->X(19);
            kalman_state.disturbance_torque.x = kalman->X(20);
            kalman_state.disturbance_torque.y = kalman->X(21);
            kalman_state.disturbance_torque.z = kalman->X(22);

            kalman_state.header.stamp = ros::Time(current_time);

            drone_gnc::DroneWaypointStamped waypoint;
            waypoint.state = kalman_state;
            state_history.trajectory.push_back(waypoint);
        }

    }
    res.trajectory = state_history;
    bag.close();
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("navigation");

    DroneEKFPixhawk kalman_obj(nh);
    kalman = &kalman_obj;

    nh.getParam("/log_file", log_file_path);

    nh.getParam("period", period);

    ros::ServiceServer service = nh.advertiseService("/kalman_simu", kalmanSimu);
    ros::spin();
}