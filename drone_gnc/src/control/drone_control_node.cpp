#include "ros/ros.h"

#include "drone_gnc/FSM.h"
#include "drone_gnc/DroneState.h"
#include "drone_gnc/Waypoint.h"
#include "drone_gnc/Trajectory.h"

#include "drone_gnc/DroneControl.h"
#include "geometry_msgs/Vector3.h"

#include "drone_gnc/GetFSM.h"
#include "drone_gnc/GetWaypoint.h"

#include <time.h>

#include <iostream>
#include <chrono>
#include <numeric>
#include <mpc/drone_mpc.h>

class DroneControlNode {
public:
    // Callback function to store last received state
    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
        current_state.pose = rocket_state->pose;
        current_state.twist = rocket_state->twist;
        current_state.thrust_scaling = rocket_state->thrust_scaling;
        received_state = true;
    }

    // Callback function to store last received state
    void targetCallback(const geometry_msgs::Vector3 &target) {
        target_apogee = target;
    }

    DroneControlNode(ros::NodeHandle &nh) : drone_mpc(nh) {
        initTopics(nh);

        std::vector<double> initial_target_apogee;
        nh.getParam("target_apogee", initial_target_apogee);
        target_apogee.x = initial_target_apogee.at(0);
        target_apogee.y = initial_target_apogee.at(1);
        target_apogee.z = initial_target_apogee.at(2);
    }


    void initTopics(ros::NodeHandle &nh) {
        // Subscribers
        rocket_state_sub = nh.subscribe("/drone_state", 100, &DroneControlNode::stateCallback, this);
        target_sub = nh.subscribe("/target_apogee", 100, &DroneControlNode::targetCallback, this);

        // Publishers
        MPC_horizon_pub = nh.advertise<drone_gnc::Trajectory>("/mpc_horizon", 10);
        drone_control_pub = nh.advertise<drone_gnc::DroneControl>("/drone_control", 10);

        // Service clients
        client_waypoint = nh.serviceClient<drone_gnc::GetWaypoint>("/getWaypoint");
    }

    void computeControl() {
        time_compute_start = ros::Time::now().toSec();

        DroneMPC::state x0;
        x0 << current_state.pose.position.x / 100, current_state.pose.position.y / 100,
                current_state.pose.position.z / 100,
                current_state.twist.linear.x / 100, current_state.twist.linear.y / 100,
                current_state.twist.linear.z / 100,
                current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
                current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

        drone_mpc.drone->setThrustScaling(current_state.thrust_scaling);

        drone_mpc.solve(x0);

        //TODO
        if (false) {
            ROS_ERROR("MPC ISSUE, SWITCHED TO PD LAW \n");
//                mpc.x_guess(x0.replicate(13, 1));
        }

    }

    void publishFeedforwardControl() {
        drone_gnc::DroneControl drone_control = drone_mpc.interpolateControlSplineService();
        drone_control_pub.publish(drone_control);
    }

    void publishTrajectory() {

        // Send optimal trajectory computed by control. Send only position for now
        drone_gnc::Trajectory trajectory_msg;
        for (int i = 0; i < drone_mpc.mpc.ocp().NUM_NODES; i++) {
            drone_gnc::Waypoint point;
            point.time = drone_mpc.mpc.time_grid(i);
            point.position.x = 100 * drone_mpc.mpc.solution_x_at(i)(0);
            point.position.y = 100 * drone_mpc.mpc.solution_x_at(i)(1);
            point.position.z = 100 * drone_mpc.mpc.solution_x_at(i)(2);
//                ROS_INFO_STREAM(100 * mpc.solution_x_at(i)(2) << " " << 100 * mpc.solution_x_at(mpc.time_grid(i))(2));
            trajectory_msg.trajectory.push_back(point);
        }
        MPC_horizon_pub.publish(trajectory_msg);

    }


    void fetchNewTarget(){
        DroneMPC::state target_state;
        DroneMPC::control target_control;

        if (client_waypoint.call(srv_waypoint)) {
            target_state << srv_waypoint.response.target_point.position.x * 1e-2,
                    srv_waypoint.response.target_point.position.y * 1e-2,
                    srv_waypoint.response.target_point.position.z * 1e-2,
                    srv_waypoint.response.target_point.speed.x * 1e-2, srv_waypoint.response.target_point.speed.y *
                                                                       1e-2,
                    srv_waypoint.response.target_point.speed.z * 1e-2,
                    current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
                    current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

            target_control << 0, 0, 0, 0;
        } else {
            target_state << target_apogee.x * 1e-2, target_apogee.y * 1e-2, target_apogee.z * 1e-2,
                    0, 0, 0,
                    0, 0, 0, 1,
                    0, 0, 0;
            target_control << 0, 0, 0, 0;
        }

        drone_mpc.setTarget(target_state, target_control);
    }

    void saveDebugInfo(){
        double x_error = current_state.pose.position.x - target_apogee.x * 100;
        average_x_error.push_back(x_error * x_error);
        double y_error = current_state.pose.position.y -  target_apogee.y * 100;
        average_y_error.push_back(y_error * y_error);
        double z_error = current_state.pose.position.z - target_apogee.z * 100;
        average_z_error.push_back(z_error * z_error);

        double compute_time = 1000 * (ros::Time::now().toSec() - time_compute_start);
        ROS_INFO("Ctr T= %.2f ms", compute_time);

        average_status.push_back(drone_mpc.mpc.info().status.value);
        average_time.push_back(time_compute_start);
    }

    void printDebugInfo(){
        std::cout << "Average time: "
                  << (std::accumulate(average_time.begin(), average_time.end(), 0.0)) / average_time.size()
                  << "ms | Average error (x, y, z): "
                  << (std::accumulate(average_x_error.begin(), average_x_error.end(), 0.0)) / average_z_error.size()
                  << ", "
                  << (std::accumulate(average_y_error.begin(), average_y_error.end(), 0.0)) / average_z_error.size()
                  << ", "
                  << (std::accumulate(average_z_error.begin(), average_z_error.end(), 0.0)) / average_z_error.size()
                  << "\n";
    }
    bool received_state = false;

private:
    DroneMPC drone_mpc;

    drone_gnc::DroneState current_state;
    geometry_msgs::Vector3 target_apogee;
    drone_gnc::GetWaypoint srv_waypoint;

    ros::Subscriber rocket_state_sub;
    ros::Subscriber target_sub;

    // Publishers
    ros::Publisher MPC_horizon_pub;
    ros::Publisher drone_control_pub;

    // Service clients
    ros::ServiceClient client_waypoint;

    // Variables to track performance over whole simulation
    double time_compute_start;
    std::vector<float> average_time;
    std::vector<int> average_status;
    std::vector<double> average_x_error;
    std::vector<double> average_y_error;
    std::vector<double> average_z_error;


};


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("control");

    DroneControlNode droneControlNode(nh);

    double mpc_period, feedforward_period;
    nh.getParam("/mpc/mpc_period", mpc_period);
    nh.getParam("/mpc/feedforward_period", feedforward_period);

    int max_ff_index = std::round(mpc_period / feedforward_period) - 1;

    ros::ServiceClient client_fsm = nh.serviceClient<drone_gnc::GetFSM>("/getFSM_gnc");

    drone_gnc::GetFSM srv_fsm;

    int ff_index = 0;
    // Thread to feedforward spline control interpolations
    ros::Timer low_level_control_thread = nh.createTimer(
            ros::Duration(feedforward_period), [&](const ros::TimerEvent &) {
                if (ff_index < max_ff_index) {
                    droneControlNode.publishFeedforwardControl();
                    ff_index++;
                }
            });

    drone_gnc::FSM current_fsm;
    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(mpc_period), [&](const ros::TimerEvent &) {

        // Get current FSM and time
        if (client_fsm.call(srv_fsm)) {
            current_fsm = srv_fsm.response.fsm;
        }

        droneControlNode.fetchNewTarget();

        // State machine ------------------------------------------
        if ((current_fsm.state_machine.compare("Idle") == 0 || current_fsm.state_machine.compare("Launch") == 0) &&
            droneControlNode.received_state) {

            droneControlNode.computeControl();

            if (current_fsm.state_machine.compare("Launch") == 0) {
                low_level_control_thread.stop();
                droneControlNode.publishFeedforwardControl();
                ff_index = 0;
                low_level_control_thread.start();
            }

            droneControlNode.publishTrajectory();

            droneControlNode.saveDebugInfo();

        } else if (current_fsm.state_machine.compare("Coast") == 0) {
            // Slow down control to 10s period for reducing useless computation
            control_thread.setPeriod(ros::Duration(10.0));

            droneControlNode.printDebugInfo();

        }
    });

    // Start spinner on a different thread to allow spline evaluation while a new solution is computed
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();

}
