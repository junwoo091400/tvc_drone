#pragma once

#include <ros/ros.h>
#include "drone_optimal_control/guidance/drone_guidance_settings.hpp"

GuidanceSettings<double> loadGuidanceSettings(ros::NodeHandle& nh)
{
  GuidanceSettings<double> settings;

  double max_attitude_angle_degree;
  if (nh.getParam("mpc/min_z", settings.min_z) && nh.getParam("mpc/min_dz", settings.min_dz) &&
      nh.getParam("mpc/max_dz", settings.max_dz) && nh.getParam("mpc/horizontal_slack", settings.horizontal_slack) &&
      nh.getParam("mpc/max_attitude_angle", max_attitude_angle_degree) &&
      nh.getParam("mpc/descent_min_propeller_speed", settings.descent_min_propeller_speed) &&
      nh.getParam("mpc/max_sqp_iter", settings.max_sqp_iter) &&
      nh.getParam("mpc/max_line_search_iter", settings.max_line_search_iter) &&
      nh.getParam("mpc/max_qp_iter", settings.max_qp_iter) &&
      nh.getParam("mpc/max_horizon_length", settings.max_horizon_length) &&
      nh.getParam("target_apogee", settings.target_apogee_vec) && nh.getParam("target_land", settings.target_land_vec))
  {
    settings.max_attitude_angle = max_attitude_angle_degree * M_PI / 180.0;
  }
  else
  {
    ROS_ERROR("Failed to get Guidance MPC parameter");
  }

  return settings;
}