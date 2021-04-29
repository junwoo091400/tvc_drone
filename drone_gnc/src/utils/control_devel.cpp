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

static const int NX = 13;
static const int NU = 4;

using namespace Eigen;

// Autodiff dor state
template<typename scalar_t>
using state_t = Matrix<scalar_t, NX, 1>;
using state = state_t<double>;

template<typename scalar_t>
using control_t = Matrix<scalar_t, NU, 1>;
using control = control_t<double>;

using ad_state = AutoDiffScalar<state>;
using ad_control = AutoDiffScalar<control>;


int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("util");
    Drone drone;
    drone.init(nh);

    state x_bar;
    x_bar << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    control u_bar;
    u_bar << 0.0, 0.0, drone.getHoverSpeedAverage(), 0.0;

    Matrix<double, 4, 1> params;
    params << 1.0, 0.0, 0.0, 0.0;

    /** initialize derivatives */
    state_t<ad_state> ADx;
    int div_size = ADx.size();
    int derivative_idx = 0;
    for (int i = 0; i < ADx.size(); ++i) {
        ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
        derivative_idx++;
    }

    //propagate xdot autodiff scalar at current x
    ADx = x_bar;
    state_t<ad_state> Xdot;
    drone.state_dynamics(ADx,
                         control_t<ad_state>(u_bar),
                         Matrix<ad_state, 4, 1>(params),
                         Xdot);

    // obtain the jacobian of f(x)
    Matrix<double, NX, NX> DF_DX;
    for (int i = 0; i < Xdot.size(); i++) {
        DF_DX.row(i) = Xdot(i).derivatives();
    }

    /** initialize derivatives */
    control_t<ad_control> ADu;
    int div_size2 = ADu.size();
    int derivative_idx2 = 0;
    for (int i = 0; i < ADu.size(); ++i) {
        ADu(i).derivatives() = control::Unit(div_size2, derivative_idx2);
        derivative_idx2++;
    }



    state_t<ad_control> Xdot2(x_bar);

//    control_t<ad_control> ADu2;
//    ADu2 = u_bar;

    for (int i = 0; i < Xdot2.size(); i++) {
        ROS_INFO_STREAM(Xdot2(i).derivatives());
    }


    drone.state_dynamics(state_t<ad_control>(x_bar),
                         ADu,
                         Matrix<ad_control, 4, 1>(params),
                         Xdot2);
    ROS_INFO_STREAM("HEEE");


    for (int i = 0; i < Xdot2.size(); i++) {
        ROS_INFO_STREAM(Xdot2(i).derivatives());
    }

    // obtain the jacobian of f(x)
    Matrix<double, NX, NU> DF_DU;
    for (int i = 0; i < Xdot2.size(); i++) {
        DF_DU.row(i) = Xdot2(i).derivatives();
    }

    ROS_INFO_STREAM("Linearized continuous time model:");
    ROS_INFO_STREAM("A = DF/DX\n" << DF_DX);
    ROS_INFO_STREAM("B = DF/DU\n" << DF_DU);

}