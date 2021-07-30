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
#include "backup_controller.hpp"

#include <time.h>

#include <iostream>
#include <chrono>
#include <numeric>
#include <mpc/drone_mpc.h>

#define USE_BACKUP_CONTROLLER false

class DroneControlNode {
public:
    double period;

    DroneControlNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr);
    void initTopics(ros::NodeHandle &nh);
    void run();

    // Callback function to store last received state
    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state);
    // Callback function to store last received state
    void targetCallback(const geometry_msgs::Vector3 &target);

    void computeControl();
    void publishFeedforwardControl();
    void publishTrajectory();
    void fetchNewTarget();

    void targetTrajectoryCallback(const drone_gnc::DroneTrajectory::ConstPtr &target);
    Drone::state sampleTargetTrajectory(double t);
    void sampleTargetTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_traj);
    void sampleTargetTrajectoryLinear(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_traj);

    void saveDebugInfo();
    void printDebugInfo();
    void publishDebugInfo();

private:
    DroneMPC drone_mpc;
    std::shared_ptr<Drone> drone;

    bool received_state = false;
    drone_gnc::DroneState current_state;
    geometry_msgs::Vector3 target_apogee;
    drone_gnc::GetWaypoint srv_waypoint;
    bool fixed_guidance;
    bool no_guidance;

    int ff_index = 0;
    // Thread to feedforward spline control interpolations
    ros::Timer low_level_control_thread;
    double feedforward_period;
    drone_gnc::FSM current_fsm;

    ros::Timer fsm_update_thread;

    //guidance trajectory variables
    //TODO find values automatically
    static const int GUIDANCE_POLY_ORDER = 7;
    static const int GUIDANCE_NUM_SEG = 2;
    static const int GUIDANCE_NUM_NODE = GUIDANCE_POLY_ORDER*GUIDANCE_NUM_SEG+1;
    //TODO swap automatically between the two
//    static const int GUIDANCE_NUM_NODE = 800;
    Eigen::Matrix<double, Drone::NX, GUIDANCE_NUM_NODE> guidanceTrajectory;
    Eigen::Matrix<double, GUIDANCE_POLY_ORDER + 1, GUIDANCE_POLY_ORDER + 1> m_basis;
    bool received_trajectory = false;
    double guidanceTraj_t0;
    double guidanceTraj_tf;

    double sgm_length;

    DroneBackupController backupController;

    ros::Subscriber rocket_state_sub;
    ros::Subscriber target_sub;
    ros::Subscriber target_traj_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;
    ros::Publisher drone_control_pub;
    drone_gnc::GetFSM srv_fsm;

    // Service clients
    ros::ServiceClient client_waypoint;
    ros::ServiceClient client_fsm;

    // Debug
    ros::Publisher sqp_iter_pub;
    ros::Publisher qp_iter_pub;
    ros::Publisher horizon_pub;
    ros::Publisher computation_time_pub;

    // Variables to track performance over whole simulation
    double time_compute_start;
    std::vector<float> average_time;
    std::vector<int> average_status;
    std::vector<double> average_x_error;
    std::vector<double> average_y_error;
    std::vector<double> average_z_error;
};