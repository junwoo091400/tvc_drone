/*
* Node to send control commands to the rocket engine.
* Can also be used by the simulation for SIL and PIL tests.
*
* Inputs:
*   - Finite state machine from the lqr_fsm :	    /gnc_fsm_pub
*   - Estimated state from lqr_navigation:		      /kalman_rocket_state
*
* Important parameters:
*   - Rocket model: 		  /config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
*	  - P gain: 		        P_control()
*   - Control loop period control_thread()
*
* Outputs:
*   - Commanded angles and thrust for the rocket gimbal:  /gimbal_command_0
*
*/

#include "ros/ros.h"

#include "../include/lqr_control.hpp"

#include <memory>
#include "rocket_types/ros_conversions.h"
#include "ros_config_loader.h"
#include "ros/package.h"
#include "drone_optimal_control/DroneTrajectory.h"
#include "drone_optimal_control/DroneExtendedState.h"



class RocketControlNode {
public:
    double frequency = 50;

    RocketControlNode() :
            rocket_fsm(RocketFSMState::IDLE) {
        ros::NodeHandle nh("~");

        // Load the rocket properties from the ROS parameter server
        DroneProps<double> drone_props = loadDroneProps(nh);

        // Instantiate the controller
         std::stringstream QR_file_path;
        QR_file_path << ros::package::getPath("lqr_control") << "/config/QR.yaml";
        controller = std::unique_ptr<LqrController>(new LqrController(drone_props, 1/frequency, QR_file_path.str()));
        
        set_point.orientation.w = 1.0;
        nh.param("track_mpc", track_mpc, false);
        std::string tracking_type;
        nh.getParam("tracking_type", tracking_type);
        if(tracking_type == "state") track_state = true;
        else track_state = false;
        // Initialize publishers and subscribers
        initTopics(nh);
    }

    void initTopics(ros::NodeHandle &nh) {
        // Create control publishers
        gimbal_command_pub = nh.advertise<rocket_utils::DroneGimbalControl>("/drone_gimbal_command_0", 10);
   

        // Subscribe to state message from basic_gnc
        bool use_ground_truth_state;
        nh.param<bool>("use_ground_truth_state", use_ground_truth_state, true);
        if (use_ground_truth_state) {
            rocket_state_sub = nh.subscribe("/rocket_state", 1, &RocketControlNode::rocketStateCallback, this);
        } else {
            rocket_state_sub = nh.subscribe("/extended_kalman_rocket_state", 1, &RocketControlNode::kalmanStateCallback, this);
        }

        // Subscribe to state message from basic_gnc
        if(track_mpc) set_point_sub = nh.subscribe("/control/debug/horizon", 1, &RocketControlNode::setPointMPCCallback, this);
        else set_point_sub = nh.subscribe("/set_point", 1, &RocketControlNode::setPointCallback, this);

        // Subscribe to fsm time_keeper
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &RocketControlNode::fsmCallback, this);
    }

    void run() {
        ros::Time time_now = ros::Time::now();

        switch (rocket_fsm) {
            case IDLE: {
                // Do nothing
                break;
            }

            // Compute attitude control and send control message
            // in both RAIL and LAUNCH mode
            case RAIL:
            case LAUNCH: {
                
                break;
            }

            case COAST:
            case STOP: {
                // Do nothing
                break;
            }
        }

        switch (rocket_fsm) {
            case IDLE: break;
            
            case RAIL:
            case ASCENT:
            // Compute roll control and send control message
            // in both LAUNCH and COAST mode
            case LAUNCH:{
                rocket_utils::DroneGimbalControl gimbal_ctrl;
                if(track_mpc){
                    double time_diff(1e99), now(ros::Time::now().toSec());
                    size_t i=0;
                    for(; i < set_point_trajectory.trajectory.size(); i++){
                        double local_time_diff = fabs(set_point_trajectory.trajectory[i].header.stamp.toSec() - now);
                        
                        if(local_time_diff <= time_diff ) time_diff = local_time_diff;
                        else if (local_time_diff >= time_diff) break;
                    }
                    if(i == 0){
                        ROS_WARN_THROTTLE(1.0, "MPC has not yet issued a set-point. Skipping...");
                        break;
                    }
                    set_point = fromROS(set_point_trajectory.trajectory[i].state.state);

                    if(track_state) gimbal_ctrl = toROS(controller->control(rocket_state, set_point));
                    else{
                        Eigen::Matrix<double,4,1> u;
                        u <<    set_point_trajectory.trajectory[i].gimbal_control.outer_angle, 
                                set_point_trajectory.trajectory[i].gimbal_control.inner_angle,
                                set_point_trajectory.trajectory[i].gimbal_control.thrust,
                                set_point_trajectory.trajectory[i].gimbal_control.torque;
                        gimbal_ctrl = toROS(controller->control(rocket_state, set_point, u));
                    }
                }
                else gimbal_ctrl = toROS(controller->control(rocket_state, set_point));
                gimbal_ctrl.header.stamp = ros::Time::now();
                gimbal_command_pub.publish(gimbal_ctrl);
                break;
            }

            case COAST: {
                
                break;
            }

            case STOP: {
                // Do nothing
                break;
            }
        }
    }

private:
    std::unique_ptr<LqrController> controller;

    // Last received rocket state
    RocketState rocket_state{}, set_point{};
    bool track_mpc;
    bool track_state;
    drone_optimal_control::DroneTrajectory set_point_trajectory;



    // Last requested fsm
    RocketFSMState rocket_fsm;

    // List of subscribers and publishers
    ros::Publisher gimbal_command_pub;
    ros::Publisher gmc_command_pub;

    ros::Subscriber rocket_state_sub;
    ros::Subscriber set_point_sub;

    ros::Subscriber fsm_sub;

    ros::Time launch_time;

    /* ------------ Callbacks functions ------------ */

    // Callback function to store last received fsm
    void fsmCallback(const rocket_utils::FSM::ConstPtr &fsm) {
        rocket_fsm = fromROS(*fsm);
        launch_time = fsm->launch_time;
    }
    // Callback function to store last received state
    void rocketStateCallback(const rocket_utils::State::ConstPtr rocket_state_msg) {
        
        rocket_state = fromROS(*rocket_state_msg);
        
    }

    void kalmanStateCallback(const drone_optimal_control::DroneExtendedState::ConstPtr rocket_state_msg) {
        rocket_state = fromROS(rocket_state_msg->state);
    }

    // Callback function to store last received state
    void setPointCallback(const rocket_utils::State::ConstPtr &set_point_msg) {
        set_point = fromROS(*set_point_msg);
        Eigen::Vector4d quat;
        quat << set_point.orientation.x, set_point.orientation.y, set_point.orientation.z, set_point.orientation.w;
        if(fabs(quat.norm() - 1.0) > 1e-3) 
        {
            ROS_WARN_STREAM("Received set-point contains orientation '" << quat << "' that is not quaternion. \nUsing unrotated quaternion (0,0,0,1) instead...");
            set_point.orientation.x = 0.;
            set_point.orientation.y = 0.;
            set_point.orientation.z = 0.;
            set_point.orientation.w = 1.;
        }
    }
    void setPointMPCCallback(const drone_optimal_control::DroneTrajectory::ConstPtr &target_trajectory) {
        this->set_point_trajectory = *target_trajectory;
        // double time_diff(1e99), now(ros::Time::now().toSec());
        // size_t i=0;
        // for(; i < target_trajectory->trajectory.size(); i++){
        //     double local_time_diff = fabs(target_trajectory->trajectory[i].header.stamp.toSec() - now);
            
        //     if(local_time_diff <= time_diff ) time_diff = local_time_diff;
        //     else if (local_time_diff >= time_diff) break;
        // }

        // ROS_ERROR_STREAM("" << i);
        // this->set_point = fromROS(target_trajectory->trajectory[i].state.state);
        // return;        
        // double now(ros::Time::now().toSec());
        // size_t i=0;
        // Eigen::Vector3d diff;
        // for(; i < target_trajectory->trajectory.size()-1; i++){
        //     diff <<     target_trajectory->trajectory[i].state.state.pose.position.x - this->rocket_state.position.x,
        //                 target_trajectory->trajectory[i].state.state.pose.position.y - this->rocket_state.position.y,
        //                 target_trajectory->trajectory[i].state.state.pose.position.z - this->rocket_state.position.z;
            
            
        //     if(diff.norm() > 5e-3) break;
           
        // }

        // ROS_ERROR_STREAM("" << i);
        // this->set_point = fromROS(target_trajectory->trajectory[i].state.state);
        // return;
    }
};

int main(int argc, char **argv) {
    // Init ROS control node
    ros::init(argc, argv, "control");

    RocketControlNode control_node;

    ros::Rate loop_rate(control_node.frequency);

    while (ros::ok()) {
        ros::spinOnce();
        control_node.run();

        loop_rate.sleep();
    }
}