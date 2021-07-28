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
    bool fixed_guidance;
    bool no_guidance;

    int ff_index = 0;
    // Thread to feedforward spline control interpolations
    ros::Timer low_level_control_thread;
    double period;
    double feedforward_period;

    ros::ServiceClient client_fsm;
    drone_gnc::GetFSM srv_fsm;
    drone_gnc::FSM current_fsm;
    ros::Timer fsm_update_thread;

    DroneControlNode(ros::NodeHandle &nh, std::shared_ptr<Drone> drone_ptr) : drone_mpc(nh, drone_ptr),
                                                                              drone(drone_ptr),
                                                                              backupController(drone_ptr) {
        initTopics(nh);

        std::vector<double> initial_target_apogee;
        nh.getParam("target_apogee", initial_target_apogee);
        target_apogee.x = initial_target_apogee.at(0);
        target_apogee.y = initial_target_apogee.at(1);
        target_apogee.z = initial_target_apogee.at(2);

        //init segment length
        double guidance_horizon_length;
        nh.getParam("guidance/mpc/horizon_length", guidance_horizon_length);
        sgm_length = guidance_horizon_length / GUIDANCE_NUM_SEG;

        nh.param("/fixed_guidance", fixed_guidance, false);
        nh.param("no_guidance", no_guidance, false);

        nh.getParam("mpc/mpc_period", period);
        nh.getParam("mpc/feedforward_period", feedforward_period);

        int max_ff_index = std::round(period / feedforward_period) - 1;

        low_level_control_thread = nh.createTimer(
                ros::Duration(feedforward_period), [&](const ros::TimerEvent &) {
                    if (ff_index < max_ff_index) {
                        publishFeedforwardControl();
                        ff_index++;
                    }
                });


        client_fsm = nh.serviceClient<drone_gnc::GetFSM>("/getFSM_gnc");

        // Initialize fsm
        current_fsm.time_now = 0;
        current_fsm.state_machine = "Idle";


        // Get current FSM and time
        fsm_update_thread = nh.createTimer(
                ros::Duration(0.2), [&](const ros::TimerEvent &) {
                    if (client_fsm.call(srv_fsm)) {
                        current_fsm = srv_fsm.response.fsm;
                    }
                });
    }

    void run(){
        double loop_start_time = ros::Time::now().toSec();
        // State machine ------------------------------------------
        if (received_trajectory || no_guidance) {

            fetchNewTarget();
            if (!USE_BACKUP_CONTROLLER) {
                computeControl();
            }

            publishTrajectory();
            saveDebugInfo();
            publishDebugInfo();

            if (current_fsm.state_machine.compare("Launch") == 0) {
                low_level_control_thread.stop();
                publishFeedforwardControl();
                ff_index = 0;
                low_level_control_thread.start();
            }

        }
        ROS_INFO_STREAM("loop period " << 1000 * (ros::Time::now().toSec() - loop_start_time) << " ms");
    }


    // Callback function to store last received state
    void stateCallback(const drone_gnc::DroneState::ConstPtr &rocket_state) {
        current_state = *rocket_state;
        received_state = true;
    }

    // Callback function to store last received state
    void targetCallback(const geometry_msgs::Vector3 &target) {
        target_apogee = target;
    }

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

    void targetTrajectoryCallback(const drone_gnc::DroneTrajectory::ConstPtr &target) {
        if (!received_trajectory && !fixed_guidance) {
            // init lagrange basis
            double t0 = target->trajectory.at(0).header.stamp.toSec();
            Eigen::Matrix<double, GUIDANCE_POLY_ORDER + 1, 1> time_grid;
            for (int i = 0; i < GUIDANCE_POLY_ORDER + 1; i++) {
                time_grid(i) = target->trajectory.at(i).header.stamp.toSec() - t0;
            }
            polympc::LagrangeSpline::compute_lagrange_basis(time_grid, m_basis);
        }
        received_trajectory = true;

        guidanceTraj_t0 = target->trajectory.at(0).header.stamp.toSec();

        int i = 0;
        for (auto waypoint: target->trajectory) {
            guidanceTrajectory.col(i)
                    << waypoint.state.pose.position.x, waypoint.state.pose.position.y, waypoint.state.pose.position.z,
                    waypoint.state.twist.linear.x, waypoint.state.twist.linear.y, waypoint.state.twist.linear.z,
                    waypoint.state.pose.orientation.x, waypoint.state.pose.orientation.y, waypoint.state.pose.orientation.z, waypoint.state.pose.orientation.w,
                    waypoint.state.twist.angular.x, waypoint.state.twist.angular.y, waypoint.state.twist.angular.z;
            i++;
        }
        guidanceTraj_tf =target->trajectory.at(i-1).header.stamp.toSec();
    }

    Drone::state sampleTargetTrajectory(double t) {
        Eigen::Index idx = (Eigen::Index) std::floor(t / sgm_length);
        idx = std::max(Eigen::Index(0),
                       std::min(idx, (Eigen::Index) GUIDANCE_NUM_SEG - 1)); // clip idx to stay within the spline bounds

        Drone::state state = polympc::LagrangeSpline::eval(t - idx * sgm_length,
                                                           guidanceTrajectory.block<Drone::NX, GUIDANCE_POLY_ORDER + 1>(
                                                                   0, idx * GUIDANCE_POLY_ORDER), m_basis);
        return state;
    }

    void sampleTargetTrajectory(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_traj) {
        double time_delta = ros::Time::now().toSec() - guidanceTraj_t0;

        for (int i = 0; i < DroneMPC::num_nodes; i++) {
            double t = time_delta + drone_mpc.time_grid(i);
            Eigen::Index idx = (Eigen::Index) std::floor(t / sgm_length);
            idx = std::max(Eigen::Index(0), std::min(idx, (Eigen::Index) GUIDANCE_NUM_SEG - 1));

            Drone::state state = polympc::LagrangeSpline::eval(t - idx * sgm_length, guidanceTrajectory.block<Drone::NX,
                    GUIDANCE_POLY_ORDER + 1>(0, idx * GUIDANCE_POLY_ORDER), m_basis);
            mpc_target_traj.col(i) = state.cwiseProduct(drone_mpc.ocp().x_drone_scaling_vec);
        }
    }


    void sampleTargetTrajectoryLinear(Matrix<double, Drone::NX, DroneMPC::num_nodes> &mpc_target_traj) {
        double time_now = ros::Time::now().toSec();

        for (int i = 0; i < DroneMPC::num_nodes; i++) {
            double t = time_now + drone_mpc.time_grid(i) - guidanceTraj_t0;
            //TODO remove constants
            double idx = t * GUIDANCE_NUM_NODE / (guidanceTraj_tf-guidanceTraj_t0);

            //linear interpolation
            int l_idx = floor(idx);
            l_idx = min(l_idx, GUIDANCE_NUM_NODE-1);
            Drone::state l_state = guidanceTrajectory.col(l_idx);

            int u_idx = ceil(idx);
            u_idx = min(u_idx, GUIDANCE_NUM_NODE-1);
            Drone::state u_state = guidanceTrajectory.col(u_idx);

            Drone::state state = l_state + (u_state - l_state) * (idx - l_idx);
            mpc_target_traj.col(i) = state.cwiseProduct(drone_mpc.ocp().x_drone_scaling_vec);
        }
    }


    void initTopics(ros::NodeHandle &nh) {
        // Subscribers
        rocket_state_sub = nh.subscribe("/drone_state", 100, &DroneControlNode::stateCallback, this);
        target_sub = nh.subscribe("/target_apogee", 100, &DroneControlNode::targetCallback, this);
        target_traj_sub = nh.subscribe("/guidance/horizon", 100, &DroneControlNode::targetTrajectoryCallback, this);


        // Publishers
        horizon_viz_pub = nh.advertise<drone_gnc::Trajectory>("/mpc_horizon", 10);
        drone_control_pub = nh.advertise<drone_gnc::DroneControl>("/drone_control", 10);

        // Service clients
        client_waypoint = nh.serviceClient<drone_gnc::GetWaypoint>("/getWaypoint", true);

        // Debug
        sqp_iter_pub = nh.advertise<std_msgs::Int32>("debug/sqp_iter", 10);
        qp_iter_pub = nh.advertise<std_msgs::Int32>("debug/qp_iter", 10);
        horizon_pub = nh.advertise<drone_gnc::DroneTrajectory>("debug/horizon", 10);
        computation_time_pub = nh.advertise<std_msgs::Float64>("debug/computation_time", 10);
    }

    void computeControl() {
        time_compute_start = ros::Time::now().toSec();


        Drone::state x0;
        x0 << current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
                current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
                current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
                current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

        drone_mpc.drone->setParams(current_state.thrust_scaling,
                                   current_state.torque_scaling,
                                   current_state.servo1_offset, current_state.servo2_offset,
                                   current_state.disturbance_force.x, current_state.disturbance_force.y,
                                   current_state.disturbance_force.z,
                                   current_state.disturbance_torque.x, current_state.disturbance_torque.y,
                                   current_state.disturbance_torque.z);



        drone_mpc.solve(x0);

        //TODO
        if (isnan(drone_mpc.solution_x_at(0)(0))) {
            ROS_ERROR("MPC ISSUE\n");
            DroneMPC::ocp_state x0;
            x0 << 0, 0, 0,
                    0, 0, 0,
                    0, 0, 0, 1,
                    0, 0, 0;
            drone_mpc.x_guess(x0.cwiseProduct(drone_mpc.ocp().x_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));
            DroneMPC::ocp_control u0;
            u0 << 0, 0, drone->getHoverSpeedAverage(), 0;
            drone_mpc.u_guess(u0.cwiseProduct(drone_mpc.ocp().u_scaling_vec).replicate(drone_mpc.ocp().NUM_NODES, 1));

            DroneMPC::dual_var_t dual;
            dual.setZero();
            drone_mpc.lam_guess(dual);
        }

    }

    void publishFeedforwardControl() {
        drone_gnc::DroneControl drone_control;
        if (USE_BACKUP_CONTROLLER) {
            drone_control = backupController.getControl(current_state, target_apogee);
        } else {
            drone_control = drone_mpc.getControlCurrentTime();
        }

        drone_control_pub.publish(drone_control);
    }

    void publishTrajectory() {

        // Send optimal trajectory computed by control. Send only position for now
        drone_gnc::Trajectory trajectory_msg;
        drone_gnc::DroneTrajectory horizon_msg;
        for (int i = 0; i < drone_mpc.ocp().NUM_NODES; i++) {
            Drone::state state_val = drone_mpc.solution_x_at(i);
//            Drone::state state_val = drone_mpc.ocp().targetTrajectory.col(i);

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
            state_msg_stamped.header.stamp = ros::Time::now() + ros::Duration(drone_mpc.node_time(i));
            state_msg_stamped.header.frame_id = ' ';


            horizon_msg.trajectory.push_back(state_msg_stamped);
        }
        horizon_viz_pub.publish(trajectory_msg);
        horizon_pub.publish(horizon_msg);

    }


    void fetchNewTarget() {
        Drone::state target_state;
        Drone::control target_control;

//        if (client_waypoint.call(srv_waypoint)) {
//            target_state << srv_waypoint.response.target_point.position.x * 1e-2,
//                    srv_waypoint.response.target_point.position.y * 1e-2,
//                    srv_waypoint.response.target_point.position.z * 1e-2,
//                    srv_waypoint.response.target_point.speed.x * 1e-2, srv_waypoint.response.target_point.speed.y *
//                                                                       1e-2,
//                    srv_waypoint.response.target_point.speed.z * 1e-2,
//                    current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
//                    current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;
//
//            target_control << 0, 0, 0, 0;
//        } else {
        target_state << target_apogee.x, target_apogee.y, target_apogee.z,
                0, 0, 0,
                0, 0, 0, 1,
                0, 0, 0;

        target_control << 0, 0, drone->getHoverSpeedAverage(), 0;
//        }

        drone_mpc.setTarget(target_state, target_control);

        Matrix<double, Drone::NX, DroneMPC::num_nodes> mpc_target_traj;
        if(no_guidance){
            for (int i = 0; i < DroneMPC::num_nodes; i++) {
                mpc_target_traj.col(i) = target_state.cwiseProduct(drone_mpc.ocp().x_drone_scaling_vec);
            }
        }
        else if (fixed_guidance) {
            sampleTargetTrajectoryLinear(mpc_target_traj);
        } else {
            sampleTargetTrajectory(mpc_target_traj);
        }

        //TODO separate
        drone_mpc.ocp().targetTrajectory = mpc_target_traj;
    }

    void saveDebugInfo() {
        double x_error = current_state.pose.position.x - target_apogee.x;
        average_x_error.push_back(x_error * x_error);
        double y_error = current_state.pose.position.y - target_apogee.y;
        average_y_error.push_back(y_error * y_error);
        double z_error = current_state.pose.position.z - target_apogee.z;
        average_z_error.push_back(z_error * z_error);

        ROS_INFO("Ctr T= %.2f ms", drone_mpc.last_computation_time);

        average_status.push_back(drone_mpc.info().status.value);
        average_time.push_back(time_compute_start);
    }

    void printDebugInfo() {
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
    DroneMPC drone_mpc;

    std::shared_ptr<Drone> drone;

    DroneBackupController backupController;

    drone_gnc::DroneState current_state;
    geometry_msgs::Vector3 target_apogee;
    drone_gnc::GetWaypoint srv_waypoint;

    ros::Subscriber rocket_state_sub;
    ros::Subscriber target_sub;
    ros::Subscriber target_traj_sub;

    // Publishers
    ros::Publisher horizon_viz_pub;
    ros::Publisher drone_control_pub;

    // Service clients
    ros::ServiceClient client_waypoint;

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


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("control");

    std::shared_ptr<Drone> drone = make_shared<Drone>(nh);

    DroneControlNode droneControlNode(nh, drone);

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = nh.createTimer(ros::Duration(droneControlNode.period), [&](const ros::TimerEvent &) {
        droneControlNode.run();
    });

    // Start spinner on a different thread to allow spline evaluation while a new solution is computed
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();

}
