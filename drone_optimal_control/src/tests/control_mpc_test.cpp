#include <iomanip>
#include <iostream>
#include <chrono>
#include <vector>
#include <utility>
#include <fstream>

#include "drone_optimal_control/control/drone_mpc.h"
#include "test_settings.h"
#include "matplotlibcpp.h"

using namespace Eigen;
namespace plt = matplotlibcpp;

int main(int argc, char* argv[])
{
  DroneProps<double> drone_props = getDroneProps();
  ControlMPCSettings<double> mpc_settings = getMPCSettings();

  mpc_settings.horizon_length = 2;
  mpc_settings.max_sqp_iter = 1;
  mpc_settings.max_qp_iter = 100;
  mpc_settings.max_line_search_iter = 4;

  Drone drone(drone_props);

  DroneMPC drone_mpc(&drone, mpc_settings);

  Drone::state target_state;
  Drone::control target_control;

  target_state << 0.5, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  target_control << 0, 0, drone.getHoverSpeedAverage(), 0;

  drone_mpc.setTargetState(target_state);
  drone_mpc.setTargetControl(target_control);

  Drone::state x0;
  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  for (int i = 0; i < 50; i++)
    drone_mpc.solve(x0, false);

  std::cout << "Control:" << std::endl;
  std::cout << drone_mpc.solution_u_at(0).transpose() << std::endl;

  std::cout << "State horizon:" << std::endl;
  for (int i = 0; i < DroneMPC::num_nodes; i++)
  {
    std::cout << i << ": " << drone_mpc.solution_x_at(i).transpose() << std::endl;
  }

  const int N = 100;
  Eigen::Matrix<double, Drone::NX, N> mpc_horizon;
  Eigen::Matrix<double, Drone::NU, N> mpc_control_horizon;
  Eigen::Matrix<double, 1, N> t;
  for (size_t i = 0; i < N; i++)
  {
    t(i) = mpc_settings.horizon_length * i / (N - 1);
    mpc_horizon.col(i) = drone_mpc.solution_x_at(t(i));
    mpc_control_horizon.col(i) = drone_mpc.solution_u_at(t(i));
  }

  plt::subplot(2, 2, 1);
  plt::plot(mpc_horizon.row(0).eval(), mpc_horizon.row(2).eval(), "", { { "label", "horizon" } });
  plt::axis("scaled");
  plt::legend();
  plt::xlabel("x");
  plt::ylabel("z");

  plt::subplot(2, 2, 2);
  plt::plot(t, (mpc_control_horizon.row(1) * 180 / M_PI).eval(), "", { { "label", "horizon" } });
  plt::legend();
  plt::xlabel("t");
  plt::ylabel("servo2 [deg]");

  plt::subplot(2, 2, 3);
  plt::plot(t, mpc_control_horizon.row(2).eval(), "", { { "label", "horizon" } });
  plt::legend();
  plt::xlabel("t");
  plt::ylabel("prop average");

  plt::tight_layout();

  plt::show();

  return EXIT_SUCCESS;
}
