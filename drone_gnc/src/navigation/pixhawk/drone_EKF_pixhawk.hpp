/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This source code is subject to the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. If a copy of the GNU General Public License was not distributed
 * with this file, you can obtain one at http://www.gnu.org/licenses/.
 */

#ifndef SRC_DRONE_EKF_PIXHAWK_HPP
#define SRC_DRONE_EKF_PIXHAWK_HPP

#include "ros/ros.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "../drone_EKF.hpp"

using namespace Eigen;

class DroneEKFPixhawk : public DroneEKF<DroneEKFPixhawk, 13> {
public:
    DroneEKFPixhawk(ros::NodeHandle &nh): DroneEKF(nh) {
        double x_var, dx_var, att_var, datt_var, thrust_scaling_var, torque_scaling_var, disturbance_force_var, disturbance_force_z_var, disturbance_torque_var, disturbance_torque_z_var;
        double x_optitrack_var, pixhawk_var, pixhawk_vel_var;
        bool use_gps;
        if (nh.getParam("predict_vars/x", x_var) &&
            nh.getParam("predict_vars/dx", dx_var) &&
            nh.getParam("predict_vars/att", att_var) &&
            nh.getParam("predict_vars/datt", datt_var) &&
            nh.getParam("predict_vars/thrust_scaling", thrust_scaling_var) &&
            nh.getParam("predict_vars/torque_scaling", torque_scaling_var) &&
            nh.getParam("predict_vars/disturbance_force", disturbance_force_var) &&
            nh.getParam("predict_vars/disturbance_force_z", disturbance_force_z_var) &&
            nh.getParam("predict_vars/disturbance_torque", disturbance_torque_var) &&
            nh.getParam("predict_vars/disturbance_torque_z", disturbance_torque_z_var) &&
            nh.getParam("update_vars/optitrack_x", x_optitrack_var) &&
            nh.getParam("update_vars/pixhawk", pixhawk_var) &&
            nh.getParam("update_vars/pixhawk_vel", pixhawk_vel_var) &&
            nh.getParam("use_gps", use_gps)) {

            state diag_Q;
            diag_Q << x_var, x_var, x_var,
                    dx_var, dx_var, dx_var,
                    att_var, att_var, att_var, att_var,
                    datt_var, datt_var, datt_var,
                    thrust_scaling_var,
                    torque_scaling_var,
                    disturbance_force_var, disturbance_force_var, disturbance_force_z_var,
                    disturbance_torque_var, disturbance_torque_var, disturbance_torque_z_var;
            setQdiagonal(diag_Q);

            sensor_data diag_R;
            if(use_gps){
                diag_R << pixhawk_var, pixhawk_var, pixhawk_var,
                        pixhawk_var, pixhawk_var, pixhawk_var,
                        pixhawk_var, pixhawk_var, pixhawk_var, pixhawk_var,
                        pixhawk_var, pixhawk_var, pixhawk_var;
            }
            else{
                diag_R << x_optitrack_var, x_optitrack_var, x_optitrack_var,
                        pixhawk_vel_var, pixhawk_vel_var, pixhawk_vel_var,
                        pixhawk_var, pixhawk_var, pixhawk_var, pixhawk_var,
                        pixhawk_var, pixhawk_var, pixhawk_var;
            }
            setRdiagonal(diag_R);

        } else {
            ROS_ERROR("Failed to get kalman filter parameter");
        }

    }

    template<typename T>
    void measurementModel_impl(const state_t<T> &x, sensor_data_t<T> &z) {
        z.head(13) = x.head(13);
    }
};


#endif //SRC_DRONE_EKF_PIXHAWK_HPP
