#pragma once

#include <ros/ros.h>
#include "../guidance/drone_guidance_settings.hpp"

NavigationSettings<double> loadNavigationSettings(ros::NodeHandle &nh) {
    NavigationSettings<double> settings;

    if (nh.getParam("predict_vars/x", settings.x_var) &&
        nh.getParam("predict_vars/dx", settings.dx_var) &&
        nh.getParam("predict_vars/att", settings.att_var) &&
        nh.getParam("predict_vars/datt", settings.datt_var) &&
        nh.getParam("predict_vars/thrust_scaling", settings.thrust_scaling_var) &&
        nh.getParam("predict_vars/torque_scaling", settings.torque_scaling_var) &&
        nh.getParam("predict_vars/disturbance_force", settings.disturbance_force_var) &&
        nh.getParam("predict_vars/disturbance_force_z", settings.disturbance_force_z_var) &&
        nh.getParam("predict_vars/disturbance_torque", settings.disturbance_torque_var) &&
        nh.getParam("predict_vars/disturbance_torque_z", settings.disturbance_torque_z_var) &&
        nh.getParam("update_vars/optitrack_x", settings.x_optitrack_var) &&
        nh.getParam("update_vars/pixhawk", settings.pixhawk_var) &&
        nh.getParam("update_vars/pixhawk_vel", settings.pixhawk_vel_var) &&
        nh.getParam("use_gps", settings.use_gps) &&
        nh.getParam("period", settings.period)
            ) {
        nh.getParam("initial_state", settings.initial_state);
        nh.param("init_estimated_params", settings.init_estimated_params, false);
        nh.param<double>("/rocket/estimated/thrust_scaling", settings.initial_thrust_scaling, 1);
        nh.param<double>("/rocket/estimated/torque_scaling", settings.initial_torque_scaling, 1);
        nh.param("estimate_params", settings.estimate_params, false);

    } else {
        ROS_ERROR("Failed to get Guidance MPC parameter");
    }

    return settings;
}