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

#include <time.h>

#include <iostream>
#include <chrono>
#include <numeric>
#include "guidance_mpc/drone_guidance_mpc.h"

#define USE_BACKUP_CONTROLLER false

class DroneGuidanceNode {
public:
    // Callback function to store last received state
    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
        current_state = *rocket_state;
        received_state = true;
    }

    // Callback function to store last received state
    void targetCallback(const geometry_msgs::Vector3 &target) {
        target_apogee = target;
    }

    DroneGuidanceNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone_mpc(nh, drone_ptr),
                                                                               drone(drone_ptr) {
        initTopics(nh);

        std::vector<double> initial_target_apogee;
        nh.getParam("target_apogee", initial_target_apogee);
        target_apogee.x = initial_target_apogee.at(0);
        target_apogee.y = initial_target_apogee.at(1);
        target_apogee.z = initial_target_apogee.at(2);
    }


    void initTopics(ros::NodeHandle &nh) {
        // Subscribers
        rocket_state_sub = nh.subscribe("/simu_drone_state", 100, &DroneGuidanceNode::stateCallback, this);
        target_sub = nh.subscribe("/target_apogee", 100, &DroneGuidanceNode::targetCallback, this);

        // Publishers
        horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/target_trajectory", 10);

        // Debug
        sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
        qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
        horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("horizon", 10);
        computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
    }

    void computeTrajectory() {
        time_compute_start = ros::Time::now();

        Drone::state x0;
        x0 << current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
                current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
                current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
                current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

        drone_mpc.drone->setParams(current_state.thrust_scaling,
                                   current_state.torque_scaling,
                                   current_state.servo1_offset, current_state.servo2_offset,
                                   0, 0, 0,
                                   0, 0, 0);

        drone_mpc.solve(x0);

        //TODO
        if (isnan(drone_mpc.solution_x_at(0)(0))) {
            ROS_ERROR("MPC ISSUE\n");
            DroneGuidanceMPC::ocp_state x0;
            x0 << 0, 0, 0,
                    0, 0, 0;
            drone_mpc.x_guess(x0.cwiseProduct(drone_mpc.ocp().x_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));
            DroneGuidanceMPC::ocp_control u0;
            u0 << 0, 0, drone->getHoverSpeedAverage();
            drone_mpc.u_guess(u0.cwiseProduct(drone_mpc.ocp().u_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));

            DroneGuidanceMPC::dual_var_t dual;
            dual.setZero();
            drone_mpc.lam_guess(dual);
        }


    }

    void publishTrajectory() {
        // Send optimal trajectory computed by control. Send only position for now
        drone_gnc::Trajectory trajectory_msg;
        drone_gnc::DroneTrajectory horizon_msg;

        for (int i = 0; i < drone_mpc.ocp().NUM_NODES; i++) {
            Drone::state state_val = drone_mpc.solution_x_at(i);

            drone_gnc::Waypoint point;
            point.time = drone_mpc.node_time(i);
            point.position.x = state_val(0);
            point.position.y = state_val(1);
            point.position.z = state_val(2);
//                ROS_INFO_STREAM(100 * mpc.solution_x_at(i)(2) << " " << 100 * mpc.solution_x_at(mpc.time_grid(i))(2));
            trajectory_msg.trajectory.push_back(point);

            drone_gnc::DroneState state_msg;
            state_msg.pose.position.x = state_val(0);
            state_msg.pose.position.y = state_val(1);
            state_msg.pose.position.z = state_val(2);

            state_msg.twist.linear.x = state_val(3);
            state_msg.twist.linear.y = state_val(4);
            state_msg.twist.linear.z = state_val(5);

            state_msg.pose.orientation.x = state_val(6);
            state_msg.pose.orientation.y = state_val(7);
            state_msg.pose.orientation.z = state_val(8);
            state_msg.pose.orientation.w = state_val(9);

            state_msg.twist.angular.x = state_val(10);
            state_msg.twist.angular.y = state_val(11);
            state_msg.twist.angular.z = state_val(12);

            Drone::control control_val = drone_mpc.solution_u_at(i);
            drone_gnc::DroneControl control_msg;
            control_msg.servo1 = control_val(0);
            control_msg.servo2 = control_val(1);
            control_msg.bottom = control_val(2);
            control_msg.top = control_val(3);

            drone_gnc::DroneWaypointStamped state_msg_stamped;
            state_msg_stamped.state = state_msg;
            state_msg_stamped.control = control_msg;
            state_msg_stamped.header.stamp = time_compute_start + ros::Duration(drone_mpc.node_time(i));
            state_msg_stamped.header.frame_id = ' ';

            horizon_msg.trajectory.push_back(state_msg_stamped);
        }
        horizon_viz_pub.publish(trajectory_msg);
        horizon_pub.publish(horizon_msg);
    }


    void fetchNewTarget() {
        DroneGuidanceMPC::ocp_state target_state;
        DroneGuidanceMPC::ocp_control target_control;

        target_state << target_apogee.x, target_apogee.y, target_apogee.z,
                0, 0, 0;

        target_control << 0, 0, drone->getHoverSpeedAverage();

        drone_mpc.setTarget(target_state, target_control);
    }

    void publishDebugInfo() {
        std_msgs::Int32 msg1; msg1.data = drone_mpc.info().iter;
        sqp_iter_pub.publish(msg1);
        std_msgs::Int32 msg2; msg2.data = drone_mpc.info().qp_solver_iter;
        qp_iter_pub.publish(msg2);
        std_msgs::Float64 msg3; msg3.data = drone_mpc.last_computation_time;
        computation_time_pub.publish(msg3);
    }

    bool received_state = false;

private:
    DroneGuidanceMPC drone_mpc;

    std::shared_ptr<Drone> drone;

    drone_gnc::DroneState current_state;
    geometry_msgs::Vector3 target_apogee;
    drone_gnc::GetWaypoint srv_waypoint;

    ros::Subscriber rocket_state_sub;
    ros::Subscriber target_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;

    // Debug
    ros::Publisher sqp_iter_pub;
    ros::Publisher qp_iter_pub;
    ros::Publisher horizon_pub;
    ros::Publisher computation_time_pub;
    // Variables to track performance over whole simulation
    ros::Time time_compute_start;
};


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh("guidance");

    std::shared_ptr<Drone> drone = make_shared<Drone>(nh);

    DroneGuidanceNode droneGuidanceNode(nh, drone);

    ros::ServiceClient client_fsm = nh.serviceClient<drone_gnc::GetFSM>("/getFSM_gnc");

    drone_gnc::GetFSM srv_fsm;
//
    drone_gnc::FSM current_fsm;
    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

//    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &) {
        double loop_start_time = ros::Time::now().toSec();

        if (client_fsm.call(srv_fsm)) {
            current_fsm = srv_fsm.response.fsm;
        }
        // State machine ------------------------------------------ //TODO remove || true
        if ((current_fsm.state_machine.compare("Launch") == 0 && droneGuidanceNode.received_state) || true) {

            droneGuidanceNode.fetchNewTarget();

            droneGuidanceNode.computeTrajectory();

            droneGuidanceNode.publishTrajectory();
            droneGuidanceNode.publishDebugInfo();

        } else if (current_fsm.state_machine.compare("Coast") == 0) {
            // Slow down control to 10s period for reducing useless computation
            control_thread.setPeriod(ros::Duration(10.0));


        }
        ROS_INFO_STREAM("loop period " << 1000 * (ros::Time::now().toSec() - loop_start_time) << " ms");
    });

    // Start spinner on a different thread to allow spline evaluation while a new solution is computed
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

}
