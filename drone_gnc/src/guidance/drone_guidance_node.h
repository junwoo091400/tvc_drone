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

#include <time.h>

#include <iostream>
#include "guidance_mpc/drone_guidance_mpc.h"

#define USE_BACKUP_CONTROLLER false

class DroneGuidanceNode {
public:
    double period;
    bool stopped = false;

    DroneGuidanceNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);

    void initTopics(ros::NodeHandle &nh);

    void run();

    // Callback function to store last received state
    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state);

    // Callback function to store last received state
    void targetCallback(const geometry_msgs::Vector3 &target);

    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm);

    void computeTrajectory();

    void publishTrajectory();

    void publishDebugInfo();




private:
    DroneGuidanceMPC drone_mpc;
    std::shared_ptr<Drone> drone;

    bool received_state = false;
    drone_gnc::DroneState current_state;
    drone_gnc::FSM current_fsm;
    Drone::state target_state;
    Drone::control target_control;
    Drone::state x0;
    bool started_descent = false;

    // Subscribers
    ros::Subscriber rocket_state_sub;
    ros::Subscriber target_sub;
    ros::Subscriber fsm_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;

    // Debug
    ros::Publisher sqp_iter_pub;
    ros::Publisher qp_iter_pub;
    ros::Publisher horizon_pub;
    ros::Publisher computation_time_pub;

    ros::ServiceClient set_fsm_client;
    // Variables to track performance over whole simulation
    ros::Time time_compute_start;
};
