#pragma once

#include <ros/ros.h>

#include "drone_model.hpp"

DroneProps<double> loadDroneProps(ros::NodeHandle& nh)
{
  DroneProps<double> props;

  double max_servo1_angle_degree, max_servo2_angle_degree, max_servo_rate_degree;
  if (nh.getParam("/rocket/min_propeller_speed", props.min_propeller_speed) &&
      nh.getParam("/rocket/max_propeller_speed", props.max_propeller_speed) &&
      nh.getParam("/rocket/max_propeller_delta", props.max_propeller_delta) &&
      nh.getParam("/rocket/max_servo1_angle", max_servo1_angle_degree) &&
      nh.getParam("/rocket/max_servo2_angle", max_servo2_angle_degree) &&
      nh.getParam("/rocket/max_servo_rate", max_servo_rate_degree) &&
      nh.getParam("/rocket/CM_to_thrust_distance", props.total_CM) && nh.getParam("/rocket/dry_mass", props.dry_mass) &&
      nh.getParam("/rocket/dry_I", props.dry_Inertia))
  {
    props.inertia << props.dry_Inertia[0], props.dry_Inertia[1], props.dry_Inertia[2];
    props.inertia_inv = props.inertia.cwiseInverse();
    props.dry_mass_inv = 1 / props.dry_mass;

    props.max_servo1_angle = max_servo1_angle_degree * (M_PI / 180);
    props.max_servo2_angle = max_servo2_angle_degree * (M_PI / 180);
    props.max_servo_rate = max_servo_rate_degree * (M_PI / 180);

    double servo1_offset_degree, servo2_offset_degree;
    nh.param<double>("/rocket/estimated/servo1_offset", servo1_offset_degree, 0);
    nh.param<double>("/rocket/estimated/servo2_offset", servo2_offset_degree, 0);
  }
  else
  {
    ROS_ERROR("Failed to get drone parameters");
  }

  return props;
}