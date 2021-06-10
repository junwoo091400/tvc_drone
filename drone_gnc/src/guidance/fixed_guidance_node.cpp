#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneWaypointStamped.h"
#include "drone_gnc/Waypoint.h"
#include "drone_gnc/Trajectory.h"
#include "drone_gnc/DroneTrajectory.h"

#include "drone_gnc/DroneControl.h"
#include "geometry_msgs/Vector3.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#include "drone_gnc/GetFSM.h"
#include "drone_gnc/GetWaypoint.h"

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

    constexpr static double seg_lengths[] = {0.5, 1, 1, 1, 0.64, 0.64, 1, 0.3, 1, 1.21/*ellipse*/, 0.5, 1.3, 2.97/*ellipse*/ };
    double total_length;
    double total_duration;

    DroneFixedGuidanceNode(ros::NodeHandle &nh) : speed(0.35) {
        total_length = 0;
        for (double seg_length:seg_lengths) {
            total_length += seg_length;
        }
        total_duration = total_length / speed;
        initTopics(nh);
    }


    void initTopics(ros::NodeHandle &nh) {
        // Publishers
        horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/target_trajectory", 10);
        horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("horizon", 10);
    }


    Vector3d sample_path(double s) {
        Vector3d seg_point;
        Vector3d seg_start;
        seg_start << 0, 0, 0;
        int i = 0;

        ////////// start
        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, 0, 0.5*s;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, 0, 0.5);

        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0, -s,0;
            return seg_start + seg_point;
        }
        s -= seg_lengths[i];
        i++;
        seg_start += Vector3d(0, -1, 0);


        if (s <= seg_lengths[i]) {
            s /= seg_lengths[i];
            seg_point << 0,0,0;
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

    void sample_traj(double t, drone_gnc::DroneState &state_msg) {
        Vector3d point = sample_path(t * speed);

        double eps = 1e-10;
        //finite difference to get speed
        Vector3d speed_vec = (point - sample_path(t * speed - eps)) / eps;
        if (speed_vec.norm() > 1e-5) {
            speed_vec *= speed / speed_vec.norm();
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
        drone_gnc::Trajectory trajectory_msg;
        drone_gnc::DroneTrajectory horizon_msg;

        int NUM_POINTS = 800;
        ros::Time time_compute_start = ros::Time::now();
        for (int i = 0; i < NUM_POINTS + 1; i++) {
            double t = i * total_duration / NUM_POINTS;

            drone_gnc::DroneState state_msg;
            sample_traj(t, state_msg);

            drone_gnc::DroneControl control_msg;
            control_msg.servo1 = 0;
            control_msg.servo2 = 0;
            control_msg.bottom = 0;
            control_msg.top = 0;

            drone_gnc::DroneWaypointStamped state_msg_stamped;
            state_msg_stamped.state = state_msg;
            state_msg_stamped.control = control_msg;
            state_msg_stamped.header.stamp = time_compute_start + ros::Duration(t);
            state_msg_stamped.header.frame_id = ' ';

            horizon_msg.trajectory.push_back(state_msg_stamped);

            drone_gnc::Waypoint point;
            point.time = t;
            point.position.x = state_msg.pose.position.x;
            point.position.y = state_msg.pose.position.y;
            point.position.z = state_msg.pose.position.z;

            trajectory_msg.trajectory.push_back(point);
        }
        horizon_viz_pub.publish(trajectory_msg);
        horizon_pub.publish(horizon_msg);
    }


private:
    // Publishers
    ros::Publisher horizon_viz_pub;

    // Debug
    ros::Publisher horizon_pub;
    // Variables to track performance over whole simulation
};


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    DroneFixedGuidanceNode droneGuidanceNode(nh);

    ros::ServiceClient client_fsm = nh.serviceClient<drone_gnc::GetFSM>("/getFSM_gnc");

    drone_gnc::GetFSM srv_fsm;

    drone_gnc::FSM current_fsm;
    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    // Get current FSM and time
    ros::Timer fsm_update_thread = nh.createTimer(
            ros::Duration(0.1), [&](const ros::TimerEvent &) {
                if (client_fsm.call(srv_fsm)) {
                    current_fsm = srv_fsm.response.fsm;
                }
            });
    bool published = false;
    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(0.2), [&](const ros::TimerEvent &) {

        if ((current_fsm.state_machine.compare("Launch") == 0) && !published) {
            droneGuidanceNode.publishTrajectory();
            published = true;

        } else if (current_fsm.state_machine.compare("Coast") == 0) {
            // Slow down control to 10s period for reducing useless computation
            control_thread.setPeriod(ros::Duration(10.0));


        }
    });

    // Start spinner on a different thread to allow spline evaluation while a new solution is computed
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();

}
