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

#include <Eigen/Eigen>

#define USE_BACKUP_CONTROLLER false

using namespace Eigen;

class DroneFixedGuidanceNode {
public:
    double speed;

    DroneFixedGuidanceNode(ros::NodeHandle &nh): speed(1){
        initTopics(nh);
    }


    void initTopics(ros::NodeHandle &nh) {
        // Publishers
        horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/target_trajectory", 10);
        horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("debug/horizon", 10);
    }

    double total_length = 2;

    void sample_path(double s, Vector3d& point){
        point << s, 0, 0;
    }

    void sample_traj(double t, drone_gnc::DroneState &state_msg){

        Vector3d point;
        sample_path(t*speed, point);

        state_msg.pose.position.x = point.x();
        state_msg.pose.position.y = point.y();
        state_msg.pose.position.z = point.z();

        state_msg.twist.linear.x = 0;
        state_msg.twist.linear.y = 0;
        state_msg.twist.linear.z = 0;

        state_msg.pose.orientation.x = 0;
        state_msg.pose.orientation.y = 0;
        state_msg.pose.orientation.z = 0;
        state_msg.pose.orientation.w = 0;

        state_msg.twist.angular.x = 0;
        state_msg.twist.angular.y = 0;
        state_msg.twist.angular.z = 0;
    }

    void publishTrajectory() {
        // Send optimal trajectory computed by control. Send only position for now
        drone_gnc::Trajectory trajectory_msg;
        drone_gnc::DroneTrajectory horizon_msg;

        int NUM_POINTS = 10;
        double traj_length = 1;
        double traj_duration = traj_length/speed;

        for (int i = 0; i < NUM_POINTS+1; i++) {
            double t = i*traj_duration/NUM_POINTS;

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
            state_msg_stamped.header.stamp = ros::Time::now() + ros::Duration(t);
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

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(0.2), [&](const ros::TimerEvent &) {
            droneGuidanceNode.publishTrajectory();

        if (current_fsm.state_machine.compare("Idle") == 0 || current_fsm.state_machine.compare("Launch") == 0) {

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
