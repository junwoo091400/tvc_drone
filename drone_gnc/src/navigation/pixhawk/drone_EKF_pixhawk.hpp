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

        double x_var, dx_var, att_var, datt_var, thrust_scaling_var, torque_scaling_var, servo_offset_var, disturbance_force_var, disturbance_force_z_var, disturbance_torque_var, disturbance_torque_z_var;
        double x_optitrack_var, pixhawk_var;
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
            nh.getParam("update_vars/pixhawk", pixhawk_var)) {

            Q.diagonal() << x_var, x_var, x_var,
                    dx_var, dx_var, dx_var,
                    att_var, att_var, att_var, att_var,
                    datt_var, datt_var, datt_var,
                    thrust_scaling_var,
                    torque_scaling_var,
                    servo_offset_var, servo_offset_var,
                    disturbance_force_var, disturbance_force_var, disturbance_force_z_var,
                    disturbance_torque_var, disturbance_torque_var, disturbance_torque_z_var;

            R.setIdentity();
            R *= 0.001;

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