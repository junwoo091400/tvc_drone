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
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;

DroneEKFPixhawk *kalman;

bool use_gps ;

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
    topics.push_back(std::string("/drone_state"));
    topics.push_back(std::string("/optitrack_client/Drone/optitrack_pose"));
    topics.push_back(std::string("/mavros/local_position/pose"));
    topics.push_back(std::string("/mavros/local_position/velocity_body"));
    topics.push_back(std::string("/mavros/local_position/velocity_local"));
    topics.push_back(std::string("/drone_control"));
    topics.push_back(std::string("/mavros/global_position/local"));
    topics.push_back(std::string("/gnc_fsm_pub"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    drone_gnc::DroneControl current_control, previous_control;
    Vector3d origin;
    double last_predict_time = 0;

    bool received_pixhawk = false;
    bool initialized_orientation = false;
    bool started = false;
    Quaterniond initial_orientation;

    Drone::state measured_drone_state;

    double t0 = 0;
    double current_time = 0;
    double last_drone_state = 0;
    bool update_trigger = false;
    for (rosbag::MessageInstance const &m: view) {

        drone_gnc::DroneControl::ConstPtr current_control_ptr = m.instantiate<drone_gnc::DroneControl>();
        if (current_control_ptr != NULL) {
            current_control = *current_control_ptr;
            if(!kalman->received_control){
                kalman->startParamEstimation();
            }
        }

        if (m.getTopic() == "/optitrack_client/Drone/optitrack_pose" && !use_gps) {
            geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
            if (received_pixhawk) {
                if (!initialized_orientation) {
                    initial_orientation.coeffs() << measured_drone_state.segment(6, 4);
                    initialized_orientation = true;
                }

                Vector<double, 3> raw_position;
                raw_position << -pose->pose.position.x, -pose->pose.position.y, 0;

                Vector<double, 3> absolute_position = initial_orientation._transformVector(raw_position);

                measured_drone_state.head(3) << absolute_position(0), absolute_position(1), pose->pose.position.z;
            }
        }
        else if (m.getTopic() == "/mavros/local_position/pose" && !use_gps) {
            geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
            measured_drone_state.segment(6, 4)
                    << pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w;

        }
        else if (m.getTopic() == "/mavros/local_position/velocity_local" && !use_gps) {
            geometry_msgs::TwistStamped::ConstPtr twist = m.instantiate<geometry_msgs::TwistStamped>();
            measured_drone_state.segment(3, 3) << twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z;
        }
        else if (m.getTopic() == "/mavros/global_position/local" && use_gps) {
            nav_msgs::Odometry::ConstPtr state = m.instantiate<nav_msgs::Odometry>();
            measured_drone_state.head(10)
                    << state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z,
                    state->twist.twist.linear.x, state->twist.twist.linear.y, -state->twist.twist.linear.z,
                    state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z, state->pose.pose.orientation.w;
        }
        else if (m.getTopic() == "/mavros/local_position/velocity_body") {
            geometry_msgs::TwistStamped::ConstPtr twist = m.instantiate<geometry_msgs::TwistStamped>();
            current_time = twist->header.stamp.toSec();
            if (t0 == 0) {
                t0 = current_time;
                last_predict_time = current_time;
            }
            measured_drone_state.segment(10, 3) << twist->twist.angular.x, twist->twist.angular.y, twist->twist.angular.z;
            received_pixhawk = true;
            update_trigger = true;
        }
        else if (!started && m.getTopic() == "/gnc_fsm_pub") {
            drone_gnc::FSM::ConstPtr fsm = m.instantiate<drone_gnc::FSM>();
            if (fsm->state_machine == drone_gnc::FSM::ASCENT) started = true;
        }
        else if (m.getTopic() == "/drone_state") {
            drone_gnc::DroneState::ConstPtr state = m.instantiate<drone_gnc::DroneState>();
            current_time = state->header.stamp.toSec();
            if (t0 == 0) {
                t0 = current_time;
                last_predict_time = current_time;
            }
        }

        if (!started) {
            origin = measured_drone_state.segment(0, 3);
        }

//        ROS_ERROR_STREAM(current_time - t0);
        if (current_time - last_predict_time >= period*0.95) {
            DroneEKFPixhawk::sensor_data new_data = measured_drone_state;
            new_data.segment(0, 3) -= origin;

            Drone::control u;
            u << previous_control.servo1, previous_control.servo2,
                    (previous_control.bottom + previous_control.top) / 2,
                    previous_control.top - previous_control.bottom;
            previous_control = current_control;

            kalman->predictStep(current_time - last_predict_time, u);
            last_predict_time = current_time;

            if (update_trigger) {
                kalman->updateStep(new_data);
                update_trigger = false;
            }

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
    nh.param("use_gps", use_gps, false);

    ros::ServiceServer service = nh.advertiseService("/kalman_simu", kalmanSimu);
    ros::spin();
}