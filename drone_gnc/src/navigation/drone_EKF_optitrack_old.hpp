/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#ifndef SRC_DRONE_EKF_OPTITRACK_OLD_HPP
#define SRC_DRONE_EKF_OPTITRACK_OLD_HPP

#include "ros/ros.h"

#include <eigen3/Eigen/Eigen>

#include "drone_EKF.hpp"

using namespace Eigen;

class DroneEKFOptitrack : public DroneEKF<DroneEKFOptitrack, 7> {
public:
    DroneEKFOptitrack(ros::NodeHandle &nh) : DroneEKF(nh) {
        double x_var, dx_var, att_var, datt_var, thrust_scaling_var, torque_scaling_var, servo_offset_var, disturbance_force_var, disturbance_force_z_var, disturbance_torque_var, disturbance_torque_z_var;
        double x_optitrack_var, att_optitrack_var;
        if (nh.getParam("predict_vars/x", x_var) &&
            nh.getParam("predict_vars/dx", dx_var) &&
            nh.getParam("predict_vars/att", att_var) &&
            nh.getParam("predict_vars/datt", datt_var) &&
            nh.getParam("predict_vars/thrust_scaling", thrust_scaling_var) &&
            nh.getParam("predict_vars/torque_scaling", torque_scaling_var) &&
            nh.getParam("predict_vars/servo_offset", servo_offset_var) &&
            nh.getParam("predict_vars/disturbance_force", disturbance_force_var) &&
            nh.getParam("predict_vars/disturbance_force_z", disturbance_force_z_var) &&
            nh.getParam("predict_vars/disturbance_torque", disturbance_torque_var) &&
            nh.getParam("predict_vars/disturbance_torque_z", disturbance_torque_z_var) &&
            nh.getParam("update_vars/optitrack_x", x_optitrack_var) &&
            nh.getParam("update_vars/optitrack_att", att_optitrack_var)) {

            state diag_Q;
            diag_Q << x_var, x_var, x_var,
                    dx_var, dx_var, dx_var,
                    att_var, att_var, att_var, att_var,
                    datt_var, datt_var, datt_var,
                    thrust_scaling_var,
                    torque_scaling_var,
                    servo_offset_var, servo_offset_var,
                    disturbance_force_var, disturbance_force_var, disturbance_force_z_var,
                    disturbance_torque_var, disturbance_torque_var, disturbance_torque_z_var;
            setQdiagonal(diag_Q);

            sensor_data diag_R;
            diag_R << x_optitrack_var, x_optitrack_var, x_optitrack_var,
                    att_optitrack_var, att_optitrack_var, att_optitrack_var, att_optitrack_var;
            setRdiagonal(diag_R);
        } else {
            ROS_ERROR("Failed to get kalman filter parameter");
        }

    }

    template<typename T>
    void measurementModel_impl(const state_t <T> &x, sensor_data_t <T> &z) {
        z.segment(0, 3) = x.segment(0, 3);
        z.segment(3, 4) = x.segment(6, 4);
    }
};


#endif //SRC_DRONE_EKF_OPTITRACK_OLD_HPP
