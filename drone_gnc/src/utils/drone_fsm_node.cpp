#include "ros/ros.h"
#include "drone_gnc/FSM.h"
#include <time.h>

#include <sstream>
#include <string>

#include "drone_gnc/GetFSM.h"
#include "std_msgs/String.h"

// global variable with time and state machine
drone_gnc::FSM current_fsm;
double time_zero;

// Service function: send back fsm (time + state machine)
bool sendFSM(drone_gnc::GetFSM::Request &req, drone_gnc::GetFSM::Response &res) {
    // Update current time
    if (current_fsm.state_machine.compare("Idle") != 0) current_fsm.time_now = ros::Time::now().toSec() - time_zero;

    res.fsm.time_now = current_fsm.time_now;
    res.fsm.state_machine = current_fsm.state_machine;

    return true;
}

void processCommand(const std_msgs::String &command) {
    if (command.data.compare("Coast") == 0) {
        current_fsm.state_machine = "Coast";
    } else {
        //received launch command
        time_zero = ros::Time::now().toSec();
        current_fsm.state_machine = "Launch";
    }
}

float rail_length = 0;

int main(int argc, char **argv) {

    // Init ROS time keeper node
    ros::init(argc, argv, "gnc_fsm");
    ros::NodeHandle nh("gnc_fsm");

    // Initialize fsm
    current_fsm.time_now = 0;
    std::string initial_state;
    nh.param<std::string>("initial_state", initial_state, "Idle");
    current_fsm.state_machine = initial_state;

    // Create timer service
    ros::ServiceServer timer_service = nh.advertiseService("/getFSM_gnc", sendFSM);

    // Create timer publisher and associated thread (100Hz)
    ros::Publisher timer_pub = nh.advertise<drone_gnc::FSM>("/gnc_fsm_pub", 10);

    // Subscribe to commands
    ros::Subscriber command_sub = nh.subscribe("/commands", 10, processCommand);

    timer_pub.publish(current_fsm);

    nh.getParam("/environment/rail_length", rail_length);

    ros::Timer FSM_thread = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent &) {
        // Update FSM
        if (current_fsm.state_machine.compare("Idle") == 0) {
        } else {
            // Update current time
            current_fsm.time_now = ros::Time::now().toSec() - time_zero;

            if (current_fsm.state_machine.compare("Rail") == 0) {
                current_fsm.state_machine = "Launch";

            } else if (current_fsm.state_machine.compare("Launch") == 0) {

            } else if (current_fsm.state_machine.compare("Coast") == 0) {
                // Do nothing for now
            }

            // Publish time + state machine
            timer_pub.publish(current_fsm);
        }

    });

    // Automatic callback of service and publisher from here
    ros::spin();

}