#include "ros/ros.h"

#include "dronc_gnc/FSM.h"
#include "drone_gnc/Waypoint.h"
#include "drone_gnc/Trajectory.h"

#include "drone_gnc/GetFSM.h"
#include "drone_gnc/GetWaypoint.h"

#include <time.h>

#include "utils/helpers.hpp"
#include "std_msgs/String.h"




// Global variable with last requested fsm
drone_gnc::FSM current_fsm;

Eigen::Matrix<double, 3, 1> current_target_apogee;

// Service function: send back waypoint at requested time
bool sendWaypoint(drone_gnc::GetWaypoint::Request &req, drone_gnc::GetWaypoint::Response &res) {
    Eigen::Matrix<double, 3, 1> next_waypoint;
    next_waypoint = current_target_apogee;

    res.target_point.position.x = next_waypoint(0);
    res.target_point.position.y = next_waypoint(1);
    res.target_point.position.z = next_waypoint(2);

    return true;
}

int main(int argc, char **argv) {
    // Init ROS guidance node
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    // Setup Time_keeper client and srv variable for FSM and time synchronization
    ros::ServiceClient client_fsm = nh.serviceClient<drone_gnc::GetFSM>("/getFSM_gnc");
    drone_gnc::GetFSM srv_fsm;

    // Create waypoint service
    ros::ServiceServer waypoint_service = nh.advertiseService("/getWaypoint", sendWaypoint);

    // Create waypoint trajectory publisher
    ros::Publisher target_trajectory_pub = nh.advertise<drone_gnc::Trajectory>("/target_trajectory", 10);

    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    ros::Publisher coast_pub = nh.advertise<std_msgs::String>("/commands", 10);

    // Thread to compute guidance. Duration defines interval time in seconds
    ros::Timer guidance_thread = nh.createTimer(ros::Duration(0.3), [&](const ros::TimerEvent &) {
        // Get current FSM and time
        if (client_fsm.call(srv_fsm)) {
            current_fsm = srv_fsm.response.fsm;
        }

        // State machine ------------------------------------------
        if (current_fsm.state_machine.compare("Idle") == 0) {
            // Do nothing
        } else if (current_fsm.state_machine.compare("Launch") == 0) {
            ROS_INFO("%f", current_fsm.time_now);
            if (current_fsm.time_now < 5) current_target_apogee << 5, 0, 10;
            else if (current_fsm.time_now < 10) current_target_apogee << 5, 0, 0;
            else if (current_fsm.time_now < 15) current_target_apogee << -5, 3, 2;
            else{
                std_msgs::String coast_command;
                coast_command.data = "Coast";
                coast_pub.publish(coast_command);
            }
        } else if (current_fsm.state_machine.compare("Coast") == 0) {
        }

    });

    // Automatic callback of service and publisher from here
    ros::spin();

}