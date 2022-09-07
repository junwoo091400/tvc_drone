#pragma once

#include <ros/ros.h>
#include "drone_optimal_control/control/drone_mpc_settings.hpp"

ControlMPCSettings<double> loadMPCSettings(ros::NodeHandle& nh)
{
  ControlMPCSettings<double> settings;

  double max_attitude_angle_degree;
  if (nh.getParam("mpc/state_costs/x", settings.x_cost) && nh.getParam("mpc/state_costs/dz", settings.dx_cost) &&
      nh.getParam("mpc/state_costs/z", settings.z_cost) && nh.getParam("mpc/state_costs/dz", settings.dz_cost) &&
      nh.getParam("mpc/state_costs/att", settings.att_cost) &&
      nh.getParam("mpc/state_costs/datt", settings.datt_cost) &&
      nh.getParam("mpc/state_costs/droll", settings.droll_cost) &&
      nh.getParam("mpc/input_costs/servo", settings.servo_cost) &&
      nh.getParam("mpc/input_costs/thrust", settings.thrust_cost) &&
      nh.getParam("mpc/input_costs/torque", settings.torque_cost) &&

      nh.getParam("mpc/min_z", settings.min_z) && nh.getParam("mpc/min_dz", settings.min_dz) &&
      nh.getParam("mpc/max_dx", settings.max_dx) && nh.getParam("mpc/max_dz", settings.max_dz) &&
      nh.getParam("mpc/max_datt", settings.max_datt) && nh.getParam("mpc/scaling_x", settings.scaling_x) &&
      nh.getParam("mpc/scaling_z", settings.scaling_z) && nh.getParam("mpc/weight_scaling", settings.weight_scaling) &&
      nh.getParam("mpc/max_attitude_angle", max_attitude_angle_degree) &&
      nh.getParam("mpc/horizon_length", settings.horizon_length) &&

      nh.getParam("mpc/max_sqp_iter", settings.max_sqp_iter) &&
      nh.getParam("mpc/max_line_search_iter", settings.max_line_search_iter) &&
      nh.getParam("mpc/period", settings.period) && nh.getParam("mpc/max_qp_iter", settings.max_qp_iter))
  {
    settings.max_attitude_angle = max_attitude_angle_degree * (M_PI / 180);
  }
  else
  {
    ROS_ERROR("Failed to get MPC parameter");
  }

  return settings;
}