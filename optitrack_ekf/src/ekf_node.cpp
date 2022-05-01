/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include <std_msgs/Float64.h>

#include "extended_kalman_filter.hpp"
#include "rocket_utils/State.h"
#include <tf/transform_broadcaster.h>

class ExtendedKalmanFilterNode
{
private:
  ExtendedKalmanFilter ekf;

  geometry_msgs::PoseStamped optitrack_pose;
  bool received_optitrack = false;
  bool initialized_optitrack = false;
  Eigen::Vector3d initial_optitrack_position;
  double last_predict_time;
  double last_computation_time = 0;

  ros::Publisher ekf_pub;
  ros::Subscriber optitrack_sub;
  ros::Publisher computation_time_pub;

public:
  double period;

  ExtendedKalmanFilterNode() : ekf()
  {
    ros::NodeHandle nh("~");
    // init publishers and subscribers
    initTopics(nh);

    double pos_std, orientation_std, vel_std, ang_vel_std, optitrack_pos_std, optitrack_orientation_std;
    if (nh.getParam("predict_std/position", pos_std) && nh.getParam("predict_std/orientation", orientation_std) &&
        nh.getParam("predict_std/vel", vel_std) && nh.getParam("predict_std/angular_vel", ang_vel_std) &&
        nh.getParam("update_std/position", optitrack_pos_std) &&
        nh.getParam("update_std/orientation", optitrack_orientation_std))
    {
      ExtendedKalmanFilter::state predict_std;
      predict_std << pos_std, pos_std, pos_std, orientation_std, orientation_std, orientation_std, orientation_std,
          vel_std, vel_std, vel_std, ang_vel_std, ang_vel_std, ang_vel_std;

      ExtendedKalmanFilter::state_matrix Q;
      Q.setZero();
      Q.diagonal() << predict_std.cwiseProduct(predict_std);

      ExtendedKalmanFilter::sensor_data update_std;
      update_std << optitrack_pos_std, optitrack_pos_std, optitrack_pos_std, optitrack_orientation_std,
          optitrack_orientation_std, optitrack_orientation_std, optitrack_orientation_std;

      ExtendedKalmanFilter::sensor_matrix R;
      R.setZero();
      R.diagonal() << update_std.cwiseProduct(update_std);

      ExtendedKalmanFilter::state X0;
      X0 << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

      ekf.init(Q, R, X0);

      double frequency;
      nh.param<double>("frequency", frequency, 100);
      period = 1 / frequency;

      initial_optitrack_position << 0, 0, 0;

      last_predict_time = ros::Time::now().toSec();
    }
    else
    {
      ROS_ERROR("Failed to get kalman filter parameter");
    }
  }

  void initTopics(ros::NodeHandle& nh)
  {
    ekf_pub = nh.advertise<nav_msgs::Odometry>("/optitrack_state", 1);
    optitrack_sub = nh.subscribe("/optitrack_client/car/pose", 1, &ExtendedKalmanFilterNode::optitrackCallback, this);
    computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
  }

  void kalmanStep()
  {
    if (received_optitrack)
    {
      if (!initialized_optitrack)
      {
        initial_optitrack_position = Eigen::Vector3d(optitrack_pose.pose.position.x, optitrack_pose.pose.position.y,
                                                     optitrack_pose.pose.position.z);
        initialized_optitrack = true;
      }

      double compute_time_start = ros::Time::now().toSec();

      Eigen::Quaterniond orientation(optitrack_pose.pose.orientation.w, optitrack_pose.pose.orientation.x,
                                     optitrack_pose.pose.orientation.y, optitrack_pose.pose.orientation.z);
      Eigen::Vector3d raw_position(optitrack_pose.pose.position.x, optitrack_pose.pose.position.y,
                                   optitrack_pose.pose.position.z);
      Eigen::Vector3d position = raw_position - initial_optitrack_position;

      ExtendedKalmanFilter::sensor_data new_data;
      new_data.segment(0, 3) = position;
      new_data.segment(3, 4) = orientation.coeffs();

      double time_now = ros::Time::now().toSec();
      double dT = time_now - last_predict_time;
      last_predict_time = time_now;

      ekf.predictStep(dT);
      ekf.updateStep(new_data);

      publishState();

      last_computation_time = (ros::Time::now().toSec() - compute_time_start) * 1000;
    }
  }

  void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
  {
    optitrack_pose = *pose;
    received_optitrack = true;
  }

  void publishState()
  {
    rocket_utils::State ekf_state_msg;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist twist_msg;

    ExtendedKalmanFilter::state X = ekf.getState();

    pose_msg.position.x = X(0);
    pose_msg.position.y = X(1);
    pose_msg.position.z = X(2);

    pose_msg.orientation.x = X(3);
    pose_msg.orientation.y = X(4);
    pose_msg.orientation.z = X(5);
    pose_msg.orientation.w = X(6);

    Vector3d vel;
    vel << X(7), X(8), X(9);
    Quaterniond orientation;
    orientation.coeffs() << X(3), X(4), X(5), X(6);
    Vector3d vel_body = orientation.inverse()._transformVector(vel);

    twist_msg.linear.x = vel_body(0);
    twist_msg.linear.y = vel_body(1);
    twist_msg.linear.z = vel_body(2);

    twist_msg.angular.x = X(10);
    twist_msg.angular.y = X(11);
    twist_msg.angular.z = X(12);

    ekf_state_msg.header.stamp = ros::Time::now();
    ekf_state_msg.pose = pose_msg;
    ekf_state_msg.twist = twist_msg;
    ekf_state_msg.propeller_mass = 0;

    ekf_pub.publish(ekf_state_msg);

    std_msgs::Float64 msg3;
    msg3.data = last_computation_time;
    computation_time_pub.publish(msg3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "optitrack_ekf");

  ExtendedKalmanFilterNode ekf_node;

  ros::Rate loop_rate(ekf_node.period);

  while (ros::ok())
  {
    ros::spinOnce();
    ekf_node.kalmanStep();

    loop_rate.sleep();
  }
}