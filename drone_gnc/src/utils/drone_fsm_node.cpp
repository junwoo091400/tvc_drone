#include "ros/ros.h"
#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include <time.h>

#include <sstream>
#include <string>

#include "drone_gnc/GetFSM.h"
#include "drone_gnc/SetFSM.h"
#include "std_msgs/String.h"

// global variable with time and state machine
drone_gnc::FSM current_fsm;
double time_zero;
drone_gnc::DroneState current_state;
bool received_state = false;
geometry_msgs::Vector3 target_apogee;

void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
//    const std::lock_guard<std::mutex> lock(state_mutex);
    current_state = *rocket_state;
    received_state = true;
}

void targetCallback(const geometry_msgs::Vector3::ConstPtr &target) {
//    const std::lock_guard<std::mutex> lock(target_mutex);
    target_apogee = *target;
}


void processCommand(const std_msgs::String &command) {
    if (command.data == "stop" || command.data == "Stop") {
        current_fsm.state_machine = drone_gnc::FSM::STOP;
    } else if(current_fsm.state_machine == drone_gnc::FSM::IDLE) {
        //received launch command
        time_zero = ros::Time::now().toSec();
        current_fsm.state_machine = drone_gnc::FSM::ASCENT;
    }
}
ros::Publisher timer_pub;
bool set_fsm(drone_gnc::SetFSM::Request &req,
             drone_gnc::SetFSM::Response &res) {
    current_fsm.state_machine = req.fsm.state_machine;
    timer_pub.publish(current_fsm);
    return true;
}

float rail_length = 0;

int main(int argc, char **argv) {

    // Init ROS time keeper node
    ros::init(argc, argv, "drone_fsm");
    ros::NodeHandle nh("drone_fsm");

    // Initialize fsm
    current_fsm.time_now = 0;
    std::string initial_state;
    nh.param<std::string>("initial_state", initial_state, "Idle");
    if(initial_state == "Idle") current_fsm.state_machine = drone_gnc::FSM::IDLE;
    else if(initial_state == "Ascent") current_fsm.state_machine = drone_gnc::FSM::ASCENT;
    bool land_after_apogee;
    nh.param<bool>("land_after_apogee", land_after_apogee, false);

    // Create timer publisher and associated thread (100Hz)
    timer_pub = nh.advertise<drone_gnc::FSM>("/gnc_fsm_pub", 10);

    // Subscribe to commands
    ros::Subscriber command_sub = nh.subscribe("/commands", 10, processCommand);

    // Subscribe to commands
    ros::Subscriber target_sub = nh.subscribe("/target_apogee", 1, targetCallback);

    ros::Publisher target_pub = nh.advertise<geometry_msgs::Vector3>("/target_apogee", 10);

    ros::ServiceServer service = nh.advertiseService("set_fsm", set_fsm);

    std::vector<double> initial_target_apogee;
    if (nh.getParam("/guidance/target_apogee", initial_target_apogee) ||
        nh.getParam("/control/target_apogee", initial_target_apogee)) {
        target_apogee.x = initial_target_apogee.at(0);
        target_apogee.y = initial_target_apogee.at(1);
        target_apogee.z = initial_target_apogee.at(2);
    }


    // Subscribe to commands
    ros::Subscriber state_sub = nh.subscribe("/drone_state", 1, stateCallback);

    timer_pub.publish(current_fsm);

    nh.getParam("/environment/rail_length", rail_length);


    ros::Timer FSM_thread = nh.createTimer(ros::Duration(0.02), [&](const ros::TimerEvent &) {
        // Update FSM
        if (current_fsm.state_machine == drone_gnc::FSM::IDLE) {
        } else {
            // Update current time
            current_fsm.time_now = ros::Time::now().toSec() - time_zero;

            if (current_fsm.state_machine == drone_gnc::FSM::ASCENT) {
//                if ((abs(current_state.pose.position.z - target_apogee.z) < 0.1 || current_state.twist.linear.z < 0) && land_after_apogee && target_apogee.z != 0){
//                    current_fsm.state_machine = "Descent";
//                }
            } else if (current_fsm.state_machine == drone_gnc::FSM::DESCENT) {
//                if (current_state.pose.position.z < 0){
////                    current_fsm.state_machine = "Stop";
//                }
            }

            // Publish time + state machine
            timer_pub.publish(current_fsm);
        }

    });

    // Automatic callback of service and publisher from here
    ros::spin();

}
