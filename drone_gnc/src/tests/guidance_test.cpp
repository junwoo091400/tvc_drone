#include <iomanip>
#include <iostream>
#include <chrono>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <fstream>
#include "matplotlibcpp.h"

#include "drone_guidance.h"
#include "test_settings.h"

using namespace Eigen;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {
    bool DESCENT = false;

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
    if (DESCENT) {
        x0
                << guidance_settings.target_apogee_vec[0], guidance_settings.target_apogee_vec[1], guidance_settings.target_apogee_vec[2],
                0, 0, 0,
                0, 0, 0, 1,
                0, 0, 0;
        drone_guidance.setTargetState(drone_guidance.target_land);
        drone_guidance.setDescentConstraints();
        drone_guidance.warmStartDescent(x0);
        drone_guidance.solve(x0);
    }

    double guidance_tf = drone_guidance.solution_p()(0);

    const int N = 100;
    Eigen::Matrix<double, Drone::NX, N> guidance_trajectory;
    Eigen::Matrix<double, Drone::NU, N> guidance_control_trajectory;
    Eigen::Matrix<double, 1, N> t;
    for (size_t i; i < N; i++) {
        t(i) = guidance_tf * i / (N - 1);
        guidance_trajectory.col(i) = drone_guidance.solution_x_at(t(i));
        guidance_control_trajectory.col(i) = drone_guidance.solution_u_at(t(i));
    }

    plt::subplot(2, 2, 1);
    plt::plot(guidance_trajectory.row(0).eval(), guidance_trajectory.row(2).eval(), "", {{"label", "horizon"}});
    plt::axis("scaled");
    plt::legend();
    plt::xlabel("x");
    plt::ylabel("z");

    plt::subplot(2, 2, 2);
    plt::plot(t, (guidance_trajectory.row(5)).eval(), "", {{"label", "horizon"}});
    plt::legend();
    plt::xlabel("t");
    plt::ylabel("velocity");

    plt::subplot(2, 2, 3);
    plt::plot(t, guidance_control_trajectory.row(2).eval(), "", {{"label", "horizon"}});
    plt::legend();
    plt::xlabel("t");
    plt::ylabel("prop average");

    plt::tight_layout();

    plt::show();

    return EXIT_SUCCESS;
}

