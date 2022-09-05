#include <iomanip>
#include <iostream>
#include <chrono>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <fstream>

#include "drone_optimal_control/control/drone_mpc.h"
#include "drone_optimal_control/guidance/drone_guidance.h"
#include "test_settings.h"

using namespace Eigen;

int main(int argc, char* argv[])
{
  DroneProps<double> drone_props = getDroneProps();
  ControlMPCSettings<double> mpc_settings = getMPCSettings();
  GuidanceSettings<double> guidance_settings = getGuidanceSettings();

  mpc_settings.horizon_length = 2;
  mpc_settings.max_sqp_iter = 3;
  mpc_settings.max_qp_iter = 100;
  mpc_settings.max_line_search_iter = 4;

  // using different values for trajectory tracking than set point tracking
  mpc_settings.x_cost = 20;
  mpc_settings.dx_cost = 1;
  mpc_settings.z_cost = 2;
  mpc_settings.dz_cost = 1;
  mpc_settings.att_cost = 100;
  mpc_settings.datt_cost = 1e-4;
  mpc_settings.droll_cost = 1;
  mpc_settings.servo_cost = 0.01;
  mpc_settings.thrust_cost = 0.005;
  mpc_settings.torque_cost = 0.001;
  mpc_settings.min_dz = -1;
  mpc_settings.max_dz = 10;
  mpc_settings.max_datt = 0.6;

  guidance_settings.max_sqp_iter = 10;
  guidance_settings.max_qp_iter = 300;
  guidance_settings.max_line_search_iter = 4;

  Drone drone(drone_props);

  DroneMPC drone_mpc(&drone, mpc_settings);
  DroneGuidance drone_guidance(&drone, guidance_settings);

  Drone::state x0;
  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // Compute guidance trajectory
  drone_guidance.solve(x0);

  double dT = drone_mpc.period;
  const int N_sim = 300;
  Eigen::Matrix<double, Drone::NX, N_sim> x_sim;
  Eigen::Matrix<double, Drone::NU, N_sim - 1> u_sim;
  Drone::state x_current, x_next;
  Drone::control u;
  double current_time = 0;

  x_sim.col(0) = x0;
  u << 0, 0, drone.getHoverSpeedAverage(), 0;

  double guidance_tf = drone_guidance.solution_p()(0);

  for (size_t i = 0; i < N_sim - 1; i++)
  {
    if (x_current[2] >= guidance_settings.target_apogee_vec[2] - 0.05)
    {
      // Apogee reached: stop simulation
      u_sim.col(i) = u;
      x_sim.col(i + 1) = x_next;
      continue;
    }
    x_current = x_sim.col(i);

    // Compute target MPC trajectory from guidance
    Matrix<double, Drone::NX, DroneMPC::num_nodes> mpc_target_state_traj;
    Matrix<double, Drone::NU, DroneMPC::num_nodes> mpc_target_control_traj;
    for (size_t j = 0; j < DroneMPC::num_nodes; j++)
    {
      double t = current_time + drone_mpc.node_time(j) + drone_mpc.period;
      mpc_target_state_traj.col(j) = drone_guidance.solution_x_at(t);
      mpc_target_control_traj.col(j) = drone_guidance.solution_u_at(t);
    }
    drone_mpc.setTargetStateTrajectory(mpc_target_state_traj);
    drone_mpc.setTargetControlTrajectory(mpc_target_control_traj);

    // reduces the horizon length when close to the apogee
    drone_mpc.setMaximumHorizonLength(guidance_tf - current_time - drone_mpc.period);

    drone_mpc.solve(x_current);
    drone.state_dynamics_discrete(x_current, u, dT, x_next);
    u = drone_mpc.solution_u_at(0);  // value is taken after the step to simulate computation time delay
    u_sim.col(i) = u;
    x_sim.col(i + 1) = x_next;
    current_time += dT;
  }

  const int N = 100;
  Eigen::Matrix<double, Drone::NX, N> guidance_trajectory;
  for (size_t i = 0; i < N; i++)
  {
    double t = guidance_tf * i / (N - 1);
    guidance_trajectory.col(i) = drone_guidance.solution_x_at(t);
  }

  const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
  std::ofstream state_file("../../tests/test_results/sim_state.csv");
  state_file << x_sim.format(CSVFormat);
  std::ofstream control_file("../../tests/test_results/sim_control.csv");
  control_file << u_sim.format(CSVFormat);
  std::ofstream guidance_file("../../tests/test_results/guidance_trajectory.csv");
  guidance_file << guidance_trajectory.format(CSVFormat);

  return EXIT_SUCCESS;
}
