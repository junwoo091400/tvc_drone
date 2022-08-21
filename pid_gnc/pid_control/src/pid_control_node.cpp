/*
* Node to send control commands to the rocket engine.
* Can also be used by the simulation for SIL and PIL tests.
*
* Inputs:
*   - Finite state machine from the pid_fsm :	    /gnc_fsm_pub
*   - Estimated state from pid_navigation:		      /kalman_rocket_state
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

#include "pid_control.hpp"

#include <memory>
#include "rocket_types/ros_conversions.h"
#include "ros_config_loader.h"
#include <ros/package.h>
#include "drone_optimal_control/DroneTrajectory.h"
#include "drone_optimal_control/DroneExtendedState.h"


std::vector<std::string> CASCADE_NAMES = {"position", "velocity_linear", "angle", "velocity_angular"};
std::vector<std::string> DIMENSION_NAMES = {"X", "Y", "Z"};


class RocketControlNode {
public:
    double frequency = 20;

    // bool loadControllersFromParameterServer(
    //     ros::NodeHandle nh,
    //     PidController::ControlCascades& cc,
    //     char** cascade_names,
    //     char** dimension_names
    // ){        
    //     cc.controllers.resize(sizeof(cascade_names));
    //     cc.errors.resize(sizeof(cascade_names));
    //     cc.error_rates.resize(sizeof(cascade_names));
    //     cc.outputs.resize(sizeof(cascade_names));
        
    //     for(size_t cascade_iter = 0; cascade_iter < cc.controllers.size(); cascade_iter++){
    //         cc.errors.at(cascade_iter).resize(sizeof(dimension_names));
    //         cc.error_rates.at(cascade_iter).resize(sizeof(dimension_names));
    //         cc.outputs.at(cascade_iter).resize(sizeof(dimension_names));
    //         for(size_t dimension_iter = 0; dimension_iter < cc.error_rates.at(cascade_iter).size(); dimension_iter++){
    //             std::vector<double> gains, limits;
    //             std::stringstream param_name;
    //             param_name << "/control/PID_parameters/" << cascade_names[cascade_iter] << "/" << dimension_names[dimension_iter];
    //             nh.getParam(param_name.str().append("/gains"), gains);
    //             nh.getParam(param_name.str().append("/limits"), limits);
    //             PID ctrl(
    //                 gains.at(0),
    //                 gains.at(1),
    //                 gains.at(2),
    //                 limits.at(0),
    //                 limits.at(1),
    //                 &cc.errors.at(cascade_iter).at(dimension_iter),
    //                 &cc.error_rates.at(cascade_iter).at(dimension_iter),
    //                 &cc.outputs.at(cascade_iter).at(dimension_iter)
    //             );
                
    //             cc.controllers.at(cascade_iter).push_back(ctrl);
    //         }
    //     }
    //     return true;
    // }

    RocketControlNode() :
            rocket_fsm(RocketFSMState::IDLE) {
                
        ros::NodeHandle nh("~");

        // Load the rocket properties from the ROS parameter server
        RocketProps rocket_props = loadRocketProps(nh);
        cc.reset(new PidController::ControlCascades());
        
        std::stringstream filename;
        filename << ros::package::getPath("pid_control") << "/config/PID_parameters.yaml";
        if(!PidController::loadControllersFromFile(filename.str(), cc, CASCADE_NAMES, DIMENSION_NAMES)){
            ROS_ERROR_STREAM("PID controller could not read parameters succeddfully. Quitting...");
            ros::requestShutdown();
        }

        // Instantiate the controller
        controller = std::unique_ptr<PidController>(new PidController(rocket_props, cc, this->control_output, this->u_feed_forward));
        
        this->u_feed_forward->outer_angle = 0.;
        this->u_feed_forward->inner_angle = 0.;
        this->u_feed_forward->thrust = 0.;
        this->u_feed_forward->torque = 0.;
        
        nh.param("track_mpc", track_mpc, false);
        nh.param("track_state", track_state, false);
        

        // Initialize publishers and subscribers
        initTopics(nh);
    }

    void initTopics(ros::NodeHandle &nh) {
        // Create control publishers
        gimbal_command_pub = nh.advertise<rocket_utils::DroneGimbalControl>("/drone_gimbal_command_0", 10);
        angle_pub = nh.advertise<geometry_msgs::Vector3>("/angle", 10);

        if(track_mpc) set_point_sub = nh.subscribe("/control/debug/horizon", 1, &RocketControlNode::setPointMPCCallback, this);
        else set_point_sub = nh.subscribe("/set_point", 1, &RocketControlNode::setPointCallback, this);

        // Subscribe to state message from basic_gnc
        bool use_ground_truth_state;
        nh.param<bool>("use_ground_truth_state", use_ground_truth_state, true);
        if (use_ground_truth_state) {
            rocket_state_sub = nh.subscribe("/rocket_state", 1, &RocketControlNode::rocketStateCallback, this);
        } else {
            rocket_state_sub = nh.subscribe("/extended_kalman_rocket_state", 1, &RocketControlNode::kalmanStateCallback, this);
        }

        

        // Subscribe to fsm time_keeper
        fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &RocketControlNode::fsmCallback, this);
    }

    void control(){
        rocket_utils::DroneGimbalControl gc;
        geometry_msgs::Vector3 angle;

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
                return;
            }
            
            if(track_state){
                set_point = fromROS(set_point_trajectory.trajectory[i].state.state);
                this->controller->updateControlLaw(this->rocket_state,  set_point, 1.0 / frequency, angle);
                gc = toROS(*this->control_output);
            }
            else{
                u_feed_forward->outer_angle = set_point_trajectory.trajectory[i].gimbal_control.outer_angle;
                u_feed_forward->inner_angle = set_point_trajectory.trajectory[i].gimbal_control.inner_angle;
                u_feed_forward->thrust = set_point_trajectory.trajectory[i].gimbal_control.thrust;
                u_feed_forward->torque = set_point_trajectory.trajectory[i].gimbal_control.torque;
                this->controller->updateControlLaw(this->rocket_state,  this->set_point, 1.0 / frequency, angle, true);
                gc = toROS(*this->control_output);
            }
        }
        else{
            this->controller->updateControlLaw(this->rocket_state,  this->set_point, 1.0 / frequency, angle);
            gc = toROS(*this->control_output);
        }
        gc.header.stamp = ros::Time::now();
        gimbal_command_pub.publish(gc);
        
        
        angle_pub.publish(angle);

        
        
        //rocket_utils::ControlMomentGyro cmg(toROS(this->control_output->second));
        //cmg.header.stamp = ros::Time::now();


        // gimbal_command_pub.publish(gc);

        //gmc_command_pub.publish(cmg);

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
            case ASCENT:
            case LAUNCH: {
                control();
                break;
            }

            case COAST:
            case STOP: {
                // Do nothing
                break;
            }
        }

        switch (rocket_fsm) {
            case IDLE:
            case RAIL: {
                // Do nothing
                break;
            }

            // Compute roll control and send control message
            // in both LAUNCH and COAST mode
            case LAUNCH:
            case COAST: {
                break;
                // Compute roll control
                // RocketControlMomentGyro gmc_control = controller->computeRollControl(rocket_state);

                // // Convert to ROS message and publish
                // rocket_utils::ControlMomentGyro gmc_control_msg = toROS(gmc_control);
                // gmc_control_msg.header.stamp = time_now;
                // gmc_command_pub.publish(gmc_control_msg);
                // break;
            }

            case STOP: {
                // Do nothing
                break;
            }
        }
    }

private:
    std::shared_ptr<PidController::ControlCascades> cc;
    std::unique_ptr<PidController> controller;
    std::shared_ptr<rocket::DroneGimbalControl> control_output;
    std::shared_ptr<DroneGimbalControl> u_feed_forward;
    bool track_mpc, track_state;
    drone_optimal_control::DroneTrajectory set_point_trajectory;

    // Last received rocket state
    RocketState rocket_state{};

    // Where the rocket should go to
    RocketState set_point{};

    // Last requested fsm
    RocketFSMState rocket_fsm;

    // List of subscribers and publishers
    ros::Publisher gimbal_command_pub;
    ros::Publisher angle_pub;

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

    void setPointMPCCallback(const drone_optimal_control::DroneTrajectory::ConstPtr &target_trajectory) {
        this->set_point_trajectory = *target_trajectory;
    }
    void setPointCallback(const rocket_utils::State::ConstPtr &set_point_msg) {
        set_point = fromROS(*set_point_msg);
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