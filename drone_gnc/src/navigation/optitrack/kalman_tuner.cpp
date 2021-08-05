#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/Sensor.h"
#include "drone_gnc/KalmanSimu.h"

#include "geometry_msgs/PoseStamped.h"

#include <time.h>

#include "drone_EKF_optitrack.h"

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace Eigen;

DroneEKF* kalman;

bool received_optitrack;
bool initialized_optitrack;
Eigen::Quaterniond initial_optitrack_orientation;
Eigen::Vector3d initial_optitrack_position;

double last_predict_time;
double period;
std::string log_file_path;

geometry_msgs::Pose optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &raw_pose) {
    geometry_msgs::Pose optitrack_pose;
    optitrack_pose.position.x = -raw_pose->pose.position.x;
    optitrack_pose.position.y = -raw_pose->pose.position.y;
    optitrack_pose.position.z = raw_pose->pose.position.z;
    optitrack_pose.orientation = raw_pose->pose.orientation;
    return optitrack_pose;
}

bool kalman_simu(drone_gnc::KalmanSimu::Request &req, drone_gnc::KalmanSimu::Response &res)
{
    received_optitrack = true;
    initialized_optitrack = false;

    kalman->estimate_params = req.estimate_params;

    DroneEKF::state Q(Map<DroneEKF::state>(req.Q.data()));
    kalman->setQdiagonal(Q);
    kalman->reset();

    geometry_msgs::Pose optitrack_pose;

    drone_gnc::DroneTrajectory state_history;
    rosbag::Bag bag;
    bag.open(ros::package::getPath("drone_utils") + "/" + log_file_path, rosbag::bagmode::Read);

    std::vector <std::string> topics;
    topics.push_back(std::string("/optitrack_client/Kite/optitrack_pose"));
    topics.push_back(std::string("/drone_control"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m: view){
        drone_gnc::DroneControl::ConstPtr current_control = m.instantiate<drone_gnc::DroneControl>();
        if(current_control != NULL){
            kalman->updateCurrentControl(current_control);
        }

        geometry_msgs::PoseStamped::ConstPtr raw_pose = m.instantiate<geometry_msgs::PoseStamped>();
        if (raw_pose != NULL) {
            double current_time = raw_pose->header.stamp.toSec();
            optitrack_pose = optitrackCallback(raw_pose);

            if(!initialized_optitrack){
                last_predict_time = current_time;
                initial_optitrack_orientation = Eigen::Quaterniond(optitrack_pose.orientation.w,
                                                                   optitrack_pose.orientation.x,
                                                                   optitrack_pose.orientation.y,
                                                                   optitrack_pose.orientation.z);
                initial_optitrack_position = Eigen::Vector3d(optitrack_pose.position.x, optitrack_pose.position.y,
                                                             optitrack_pose.position.z);
                initialized_optitrack = true;
            }

            if (current_time - last_predict_time > period) {
                Eigen::Quaterniond raw_orientation(optitrack_pose.orientation.w, optitrack_pose.orientation.x,
                                                   optitrack_pose.orientation.y, optitrack_pose.orientation.z);
                Eigen::Vector3d raw_position(optitrack_pose.position.x, optitrack_pose.position.y,
                                             optitrack_pose.position.z);
                Eigen::Quaterniond orientation = initial_optitrack_orientation.inverse() * raw_orientation;
                Eigen::Vector3d position = raw_position - initial_optitrack_position;


                DroneEKF::sensor_data new_data;
                new_data.segment(0, 3) = position;
                new_data.segment(3, 4) = orientation.coeffs();

                kalman->predictStep(period);
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
    }
    res.trajectory = state_history;
    bag.close();
    return true;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("navigation");

    DroneEKF kalman_obj(nh);
    kalman = &kalman_obj;

    initial_optitrack_orientation.setIdentity();
    initial_optitrack_position << 0, 0, 0;

    nh.getParam("/log_file", log_file_path);

    nh.getParam("period", period);

    ros::ServiceServer service = nh.advertiseService("/kalman_simu", kalman_simu);
    ros::spin();
}