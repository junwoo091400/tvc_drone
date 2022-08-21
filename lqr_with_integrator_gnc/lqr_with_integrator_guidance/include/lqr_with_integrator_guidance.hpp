#pragma once

#include "rocket_types/rocket_types.h"
#include <memory>

using namespace rocket;

class Lqr_with_integratorGuidance
{
public:
  struct Config
  {
    Position target_apogee;
    double final_time = 20.0;

    Config(Position target_apogee_) : target_apogee(target_apogee_)
    {
    }
  };

  Config config;

  Lqr_with_integratorGuidance(Config config_) : config(config_)
  {
  }

  // Creates very basic trajectory to reach desired points at apogee using affine functions
  void computeTrajectory(RocketState& current_state, double time_now)
  {
    if (time_now > config.final_time)
      return;

    double dT = config.final_time - time_now;

    // Define affine parameters for position trajectory
    a_x = (config.target_apogee.x - current_state.position.x) / dT;
    a_y = (config.target_apogee.y - current_state.position.y) / dT;
    a_z = (config.target_apogee.z - current_state.position.z) / dT;

    b_x = config.target_apogee.x - a_x * config.final_time;
    b_y = config.target_apogee.y - a_y * config.final_time;
    b_z = config.target_apogee.z - a_z * config.final_time;
  }

  inline Position sampleTrajectory(double t) const
  {
    Position position;
    position.x = a_x * t + b_x;
    position.y = a_y * t + b_y;
    position.z = a_z * t + b_z;

    return position;
  }

private:
  double a_x, a_y, a_z, b_x, b_y, b_z;
};