#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/DroneControl.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "nav_msgs/Odometry.h"

#include <std_msgs/Float64.h>

#include "drone_EKF_pixhawk.hpp"

class DroneNavigationNodePixhawk {
private:
    DroneEKFPixhawk kalman;

    drone_gnc::FSM current_fsm;
    drone_gnc::DroneControl current_control;
    drone_gnc::DroneControl previous_control;
    drone_gnc::DroneState pixhawk_state;
    bool received_pixhawk = false;
    //TODO set to false
    bool received_optitrack = false;
    bool initialized_origin = false;
    bool initialized_orientation = false;
    double last_predict_time;
    double last_computation_time = 0;
    Drone::state measured_drone_state;
    Eigen::Vector3d origin;
    Eigen::Quaterniond initial_orientation;
    bool use_gps;
    bool update_trigger = false;
    double init_time;

    ros::Publisher kalman_pub;
    ros::Publisher computation_time_pub;
    ros::Subscriber fsm_sub;
    ros::Subscriber control_sub;
    ros::Subscriber optitrack_sub;
    ros::Subscriber pixhawk_pose_sub;
    ros::Subscriber pixhawk_twist_local_sub;
    ros::Subscriber pixhawk_twist_body_sub;
    ros::Subscriber pixhawk_ekf_sub;

public:
    double period;

    DroneNavigationNodePixhawk(ros::NodeHandle &nh);
    void initTopics(ros::NodeHandle &nh);

    void kalmanStep();

    // Callback function to store last received fsm
    void fsmCallback(const drone_gnc::FSM::ConstPtr &fsm);

    void pixhawkEKFCallback(const nav_msgs::Odometry::ConstPtr &state);
    // Callback function to store last received state
    void pixhawkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    // Callback function to store last received state
    void pixhawkTwistBodyCallback(const geometry_msgs::TwistStamped::ConstPtr &twist);
    // Callback function to store last received state
    void pixhawkTwistLocalCallback(const geometry_msgs::TwistStamped::ConstPtr &twist);
    // Callback function to store last received state
    void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

    void controlCallback(const drone_gnc::DroneControl::ConstPtr &control);

    void publishDroneState();
};