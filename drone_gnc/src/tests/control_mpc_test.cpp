#include <iomanip>
#include <iostream>
#include <chrono>
#include <vector>
#include <utility>
#include <eigen3/Eigen/Eigen>
#include <fstream>

#include "drone_mpc.h"
#include "test_settings.h"

using namespace Eigen;

int main(int argc, char *argv[]) {
    DroneProps<double> drone_props = getDroneProps();
    ControlMPCSettings<double> mpc_settings = getMPCSettings();

    mpc_settings.horizon_length = 2;
    mpc_settings.max_sqp_iter = 3;
    mpc_settings.max_qp_iter = 100;
    mpc_settings.max_line_search_iter = 4;

    Drone drone(drone_props);

    DroneMPC drone_mpc(&drone, mpc_settings);

    Drone::state target_state;
    Drone::control target_control;

    target_state << 0.5, 0, 1,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    target_control << 0, 0, drone.getHoverSpeedAverage(), 0;

    drone_mpc.setTargetState(target_state);
    drone_mpc.setTargetControl(target_control);

    Drone::state x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    drone_mpc.solve(x0);

    std::cout << "Control:" << std::endl;
    std::cout << drone_mpc.solution_u_at(0).transpose() << std::endl;

    std::cout << "State horizon:" << std::endl;
    for (int i = 0; i < DroneMPC::num_nodes; i++) {
        std::cout << i << ": " << drone_mpc.solution_x_at(i).transpose() << std::endl;
    }

    double dT = drone_mpc.period;
    const int N_sim = 500;
    Eigen::Matrix<double, Drone::NX, N_sim> x_sim;
    Eigen::Matrix<double, Drone::NU, N_sim - 1> u_sim;
    Drone::state x_current, x_next;
    Drone::control u;
    double current_time = 0;

    x_sim.col(0) = x0;
    u << 0, 0, drone.getHoverSpeedAverage(), 0;

    for (size_t i = 0; i < N_sim - 1; i++) {
        x_current = x_sim.col(i);
        drone_mpc.solve(x_current);
        drone.state_dynamics_discrete(x_current, u, dT, x_next);
        u = drone_mpc.solution_u_at(0); //value is taken after the step to simulate computation time delay
        u_sim.col(i) = u;
        x_sim.col(i + 1) = x_next;
        current_time += dT;
    }

    const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
    std::ofstream state_file("../../tests/test_results/sim_state.csv");
    state_file << x_sim.format(CSVFormat);
    std::ofstream control_file("../../tests/test_results/sim_control.csv");
    control_file << u_sim.format(CSVFormat);

    return EXIT_SUCCESS;
}

