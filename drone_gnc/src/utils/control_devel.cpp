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


#include "Eigen/Core"
#include "Eigen/Geometry"

#include <autodiff/AutoDiffScalar.h>
#include <drone_model.hpp>
#include "mpc_utils.hpp"

int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("util");
    Drone drone(nh);

    Matrix<double, 11, 1> Q;
    Matrix<double, Drone::NU, 1> R;
    Q << 1, 1, 5,
            0.1, 0.1, 0.5,
            2, 2,
            2, 2, 5;
    R << 5, 5, 0.01, 0.01;

    Matrix<double, NX - 2, NX - 2> QN;
    compute_LQR_terminal_cost(drone, Q, R,QN);

    ROS_INFO_STREAM("QN\n" << QN);
}