/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "ros/ros.h"

#include "rocket_utils/FSM.h"
#include "rocket_utils/ExtendedState.h"
#include "drone_optimal_control/DroneWaypointStamped.h"
#include "rocket_utils/Waypoint.h"
#include "rocket_utils/Trajectory.h"
#include "drone_optimal_control/DroneTrajectory.h"

#include "rocket_utils/GimbalControl.h"
#include "rocket_utils/ControlMomentGyro.h"
#include "geometry_msgs/Vector3.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#include <time.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <math.h>

#include <Eigen/Eigen>

#define USE_BACKUP_CONTROLLER false

using namespace Eigen;

class DroneFixedGuidanceNode {
public:
    double speed;
    rocket_utils::FSM current_fsm;

    double scaling = 1;

    constexpr static double seg_lengths[] = {0.5, 1, 1, 1, 1, 0.64, 0.64, 1, 0.3, 1, 1.21/*ellipse*/, 0.5, 1.3,
                                             2.97/*ellipse*/ };
    double total_length;
    double total_duration;
    bool published = false;
    double rotation_angle;

    DroneFixedGuidanceNode(ros::NodeHandle &nh) : speed(0.35) {
        nh.getParam("speed", speed);
        nh.getParam("scaling", scaling);

        nh.param<double>("rotation_angle", rotation_angle, 0.0);

        total_length = 0;
        for (double seg_length: seg_lengths) {
            total_length += seg_length;
        }
        total_length *= scaling;
        total_duration = total_length / speed;
        ROS_ERROR_STREAM(total_duration);
        initTopics(nh);
    }

    void run() {
        if (current_fsm.state_machine != rocket_utils::FSM::ASCENT) {
            publishTrajectory();
        }
    }


    void initTopics(ros::NodeHandle &nh) {
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &DroneFixedGuidanceNode::fsmCallback, this);
        // Publishers
        horizon_viz_pub = nh.advertise<rocket_utils::Trajectory>("/target_trajectory", 10);
        horizon_pub = nh.advertise<drone_optimal_control::DroneTrajectory>("horizon", 10);
    }

    void fsmCallback(const rocket_utils::FSM::ConstPtr &fsm) {
        current_fsm = *fsm;
    }

    Vector3d samplePathRaw(double s) {
        Vector3d seg_point;
        Vector3d seg_start;
        seg_start << 0, 0, 0;
        int i = 0;

        ////////// start
        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, 0.5 * s;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, 0.5);


        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, 0;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, -s, 0;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, -1, 0);


        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, 0;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;

        ////////// M
        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, s;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, 1);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, s * 0.4, -s * 0.5;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0.4, -0.5);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, s * 0.4, s * 0.5;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0.4, 0.5);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, -s;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, -1);

        ////////// P
        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0.3 * s, 0;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0.3, 0);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, s;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, 1);


        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            double r = 0.25;
            double theta = s * M_PI;
            seg_point << 0, r * 2 * sin(theta), -0.25 + r * cos(theta);
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, -0.5);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, -0.5 * s;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, -0.5);

        ////////// C
        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, s, 0;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 1.3, 0);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            double r = 0.5;
            double theta = s * M_PI;
            seg_point << 0, -r * 1.3 * sin(theta), 0.5 - r * cos(theta);
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, 1);

        return seg_start;
    }

    Vector3d samplePath(double t) {
        Vector3d point = samplePathRaw(t * speed) * scaling;
        AngleAxisd rot(rotation_angle * M_PI / 180, Vector3d(0, 0, 1));
        point = rot._transformVector(point);
        return point;
    }

    void sample_traj(double t, rocket_utils::State &state_msg) {
        Vector3d point = samplePath(t);

        double eps = 1e-10;
        //finite difference to get speed
        Vector3d speed_vec = (samplePath(t + eps) - point) / eps;
        if (speed_vec.norm() > 1e-5) {
            speed_vec *= speed / speed_vec.norm();
        }

        if (t >= total_duration) {
            speed_vec.setZero();
        }

        state_msg.pose.position.x = point.x();
        state_msg.pose.position.y = point.y();
        state_msg.pose.position.z = point.z();

        state_msg.twist.linear.x = speed_vec.x();
        state_msg.twist.linear.y = speed_vec.y();
        state_msg.twist.linear.z = speed_vec.z();

        state_msg.pose.orientation.x = 0;
        state_msg.pose.orientation.y = 0;
        state_msg.pose.orientation.z = 0;
        state_msg.pose.orientation.w = 1;

        state_msg.twist.angular.x = 0;
        state_msg.twist.angular.y = 0;
        state_msg.twist.angular.z = 0;
    }

    void publishTrajectory() {
        // Send optimal trajectory computed by control. Send only position for now
        rocket_utils::Trajectory trajectory_msg;
        drone_optimal_control::DroneTrajectory horizon_msg;

        int NUM_POINTS = 800;
        ros::Time time_compute_start = ros::Time::now();
        for (int i = 0; i < NUM_POINTS; i++) {
            double t = i * total_duration / (NUM_POINTS - 1);

            rocket_utils::State state_msg;
            sample_traj(t, state_msg);

            rocket_utils::DroneGimbalControl gimbal_control_msg;
            gimbal_control_msg.outer_angle = 0;
            gimbal_control_msg.inner_angle = 0;
            gimbal_control_msg.thrust = 0;
            gimbal_control_msg.torque = 0;

            

            drone_optimal_control::DroneWaypointStamped state_msg_stamped;
            state_msg_stamped.state.state = state_msg;
            state_msg_stamped.gimbal_control = gimbal_control_msg;
            state_msg_stamped.header.stamp = time_compute_start + ros::Duration(t);
            state_msg_stamped.header.frame_id = ' ';

            horizon_msg.trajectory.push_back(state_msg_stamped);

            rocket_utils::Waypoint point;
            point.time = t;
            point.position.x = state_msg.pose.position.x;
            point.position.y = state_msg.pose.position.y;
            point.position.z = state_msg.pose.position.z;

            trajectory_msg.trajectory.push_back(point);
        }
        horizon_msg.num_node = NUM_POINTS;
        horizon_viz_pub.publish(trajectory_msg);
        horizon_pub.publish(horizon_msg);
    }


private:
    ros::Subscriber fsm_sub;
    ros::Publisher horizon_viz_pub;
    ros::Publisher horizon_pub;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    DroneFixedGuidanceNode droneGuidanceNode(nh);

    ros::Timer control_thread = nh.createTimer(ros::Duration(0.2), [&](const ros::TimerEvent &) {
        droneGuidanceNode.run();
    });

    ros::spin();

}
