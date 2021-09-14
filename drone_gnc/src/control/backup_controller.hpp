/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This source code is subject to the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. If a copy of the GNU General Public License was not distributed
 * with this file, you can obtain one at http://www.gnu.org/licenses/.
 */

#include <ros/ros.h>
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/DroneState.h"
#include "geometry_msgs/Vector3.h"

#include <Eigen/Eigen>
#include "rocket_model.hpp"

#include <cmath>
#include <drone_model.hpp>

using namespace Eigen;

class DroneBackupController {
public:
    static const int NX = 12;
    static const int NU = 4;

    using state = Matrix<double, NX, 1>;
    using control = Matrix<double, NU, 1>;

    Matrix<double, 4, 11> K;
    state xs;
    control us;

    std::shared_ptr<Drone> drone;

    DroneBackupController(std::shared_ptr<Drone> drone_ptr) : drone(drone_ptr) {
        K << -0.32212, 0, 0, -0.38319, 0, 0, 0, -4.1557, 0, -0.62352, 0,
                0, 0.31557, 0, 0, 0.37408, 0, -4.0404, 0, -0.60078, 0, 0,
                0, 0, 21.602, 0, 0, 15.742, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -21.847;
        xs.setZero();
        double hover_speed = drone->getHoverSpeedAverage();
        us << 0, 0, hover_speed, 0;
    }


    drone_gnc::DroneControl getControl(drone_gnc::DroneState &x_msg, geometry_msgs::Vector3 target_apogee) {
        Drone::state x;
        x << x_msg.pose.position.x, x_msg.pose.position.y, x_msg.pose.position.z,
                x_msg.twist.linear.x, x_msg.twist.linear.y, x_msg.twist.linear.z,
                x_msg.pose.orientation.x, x_msg.pose.orientation.y, x_msg.pose.orientation.z, x_msg.pose.orientation.z,
                x_msg.twist.angular.x, x_msg.twist.angular.y, x_msg.twist.angular.z;

        Drone::state xs;
        xs << target_apogee.x, target_apogee.y, target_apogee.z,
                0, 0, 0,
                0, 0, 0, 1,
                0, 0, 0;

        Matrix<double, 13, 1> x_error = x - xs;
        Matrix<double, 11, 1> x_error2;
        x_error2.segment(0, 6) = x_error.segment(0, 6);
        x_error2(6) = x_error(9) * x_error(6) - x_error(7) * x_error(8);
        x_error2(7) = x_error(9) * x_error(7) + x_error(6) * x_error(8);
        x_error2.segment(8, 3) = x_error.segment(10, 3);

        ROS_ERROR_STREAM("qx " << x_error2(6));
        ROS_ERROR_STREAM("qy " << x_error2(7));

        control u_tilde = -K * x_error2;
        control u = u_tilde + us;

        ROS_ERROR_STREAM("servo1 "  << u(0));
        ROS_ERROR_STREAM("servo2 " << u(1));

        drone_gnc::DroneControl drone_control;
        drone_control.servo1 = u(0);
        drone_control.servo2 = u(1);
        drone_control.bottom = u(2) - u(3) / 2;
        drone_control.top = u(2) + u(3) / 2;

        drone_control.servo1 = std::min(std::max(drone_control.servo1, -drone->max_servo1_angle),
                                        drone->max_servo1_angle);
        drone_control.servo2 = std::min(std::max(drone_control.servo2, -drone->max_servo2_angle),
                                        drone->max_servo2_angle);
        drone_control.top = std::min(std::max(drone_control.top, drone->min_propeller_speed),
                                     drone->max_propeller_speed);
        drone_control.bottom = std::min(std::max(drone_control.bottom, drone->min_propeller_speed),
                                        drone->max_propeller_speed);

        return drone_control;
    };
};

