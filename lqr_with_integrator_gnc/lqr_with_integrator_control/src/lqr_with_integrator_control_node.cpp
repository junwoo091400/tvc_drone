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

#include "../include/lqr_with_integrator_control.hpp"

#include <memory>
#include "rocket_types/ros_conversions.h"
#include "ros_config_loader.h"
#include "ros/package.h"
#include "drone_optimal_control/DroneTrajectory.h"


class RocketControlNode {
public:
    double frequency = 50;

    RocketControlNode() :
            rocket_fsm(RocketFSMState::IDLE) {
        ros::NodeHandle nh("~");

        // Load the rocket properties from the ROS parameter server
        DroneProps<double> drone_props = loadDroneProps(nh);
        std::stringstream QR_file_path;
        QR_file_path << ros::package::getPath("lqr_with_integrator_control") << "/config/QR.yaml";
        // Instantiate the controller
        controller = std::unique_ptr<Lqr_with_integratorController>(new Lqr_with_integratorController(drone_props, 1/frequency, QR_file_path.str()));

        set_point.orientation.w = 1.0;
        nh.param("track_mpc", track_mpc, false);
        // Initialize publishers and subscribers
        initTopics(nh);
    }

    void initTopics(ros::NodeHandle &nh) {
        // Create control publishers
        gimbal_command_pub = nh.advertise<rocket_utils::DroneGimbalControl>("/drone_gimbal_command_0", 10);
   

        // Subscribe to state message from basic_gnc
        rocket_state_sub = nh.subscribe("/rocket_state", 1, &RocketControlNode::rocketStateCallback, this);

        // Subscribe to state message from basic_gnc
        set_point_sub = nh.subscribe("/set_point", 1, &RocketControlNode::setPointCallback, this);

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
            // Compute roll control and send control message
            // in both LAUNCH and COAST mode
            case LAUNCH:{
                rocket_utils::DroneGimbalControl gimbal_ctrl = toROS(controller->control(rocket_state, set_point));
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
    std::unique_ptr<Lqr_with_integratorController> controller;

    // Last received rocket state
    RocketState rocket_state{}, set_point{};
    bool track_mpc;


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
    void rocketStateCallback(const rocket_utils::State::ConstPtr &rocket_state_msg) {
        
        rocket_state = fromROS(*rocket_state_msg);
        
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