/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#include "ros/ros.h"

#include <time.h>
#include <iostream>

#include "Eigen/Core"
#include <drone_model.hpp>

int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("util");
    Drone drone(nh);

//    Matrix<double, 11, 1> Q;
//    Matrix<double, Drone::NU, 1> R;
//    Q << 1, 1, 5,
//            0.1, 0.1, 0.5,
//            2, 2,
//            2, 2, 5;
//    R << 5, 5, 0.01, 0.01;
//
//    Matrix<double, NX - 2, NX - 2> QN;
//    compute_LQR_terminal_cost(drone, Q, R, QN);

//    ROS_INFO_STREAM("QN\n" << QN);
}