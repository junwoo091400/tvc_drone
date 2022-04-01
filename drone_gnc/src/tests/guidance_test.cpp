#include <iomanip>
#include <iostream>
#include <chrono>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <fstream>

#include "drone_guidance.h"
#include "test_settings.h"

using namespace Eigen;

int main(int argc, char *argv[]) {
    DroneProps<double> drone_props = getDroneProps();
    GuidanceSettings<double> guidance_settings = getGuidanceSettings();

    guidance_settings.max_sqp_iter = 10;
    guidance_settings.max_qp_iter = 300;
    guidance_settings.max_line_search_iter = 4;

    Drone drone(drone_props);

    DroneGuidance drone_guidance(&drone, guidance_settings);

    Drone::state x0;

    // Ascent
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    drone_guidance.solve(x0);

    // Descent
    x0
            << guidance_settings.target_apogee_vec[0], guidance_settings.target_apogee_vec[1], guidance_settings.target_apogee_vec[2],
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    drone_guidance.setTargetState(drone_guidance.target_land);
    drone_guidance.setDescentConstraints();
    drone_guidance.warmStartDescent(x0);
    drone_guidance.solve(x0);

    double guidance_tf = drone_guidance.solution_p()(0);

    // Save trajectory to csv
    const int N = 100;
    Eigen::Matrix<double, Drone::NX, N> guidance_trajectory;
    for (size_t i; i < N; i++) {
        double t = guidance_tf * i / (N - 1);
        guidance_trajectory.col(i) = drone_guidance.solution_x_at(t);
    }

    const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
    std::ofstream guidance_file("../../tests/test_results/guidance_trajectory.csv");
    guidance_file << guidance_trajectory.format(CSVFormat);


    return EXIT_SUCCESS;
}

