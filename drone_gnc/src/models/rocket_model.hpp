#ifndef SRC_ROCKET_MODEL_HPP
#define SRC_ROCKET_MODEL_HPP

#include <Eigen/Eigen>
#include <fstream>
#include <ros/ros.h>

class Rocket {
public:
    double dry_mass;
    double dry_mass_inv;

    double total_CM; // Current Cm of rocket, in real time

    std::vector<double> dry_Inertia{0, 0, 0};

    Eigen::Vector<double, 3> gravity_vector;
    Eigen::Vector<double, 3> inertia;
    Eigen::Vector<double, 3> inertia_inv;

//    Eigen::Vector<double, 3> euler_coefs;

    Rocket(ros::NodeHandle n) {
        if (n.getParam("/rocket/dry_mass", dry_mass) &&
            n.getParam("/rocket/dry_I", dry_Inertia)) {}
        else {
            ROS_ERROR("Failed to get rocket parameters");
        }

        inertia << dry_Inertia[0], dry_Inertia[1], dry_Inertia[2];
        inertia_inv = inertia.cwiseInverse();
        dry_mass_inv = 1 / dry_mass;
        // Force in inertial frame: gravity
        double g0 = 9.81;
        gravity_vector << 0, 0, -g0 * dry_mass;

//        euler_coefs << (inertia[2] - inertia[1]) / inertia[0],
//                (inertia[0] - inertia[2]) / inertia[1],
//                (inertia[1] - inertia[0]) / inertia[2];
    }


    template<typename T, typename state>
    inline void generic_rocket_dynamics(const Eigen::Matrix<T, 13, 1> x,
                                        const Eigen::Matrix<T, 3, 1> thrust_vector,
                                        const Eigen::Matrix<T, 3, 1> body_torque,
                                        const Eigen::Matrix<T, 3, 1> disturbance_force,
                                        const Eigen::Matrix<T, 3, 1> disturbance_torque,
                                        state &xdot) const noexcept {
        // -------------- Simulation parameters -------------- -------------
        // Earth gravity in [m/s^2]

        // -------------- Simulation variables -----------------------------
        T mass_inv = (T) dry_mass_inv;

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));

        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Total force in inertial frame [N]
        Eigen::Vector<T, 3> total_force;
        total_force = rot_matrix * thrust_vector + gravity_vector.template cast<T>() + disturbance_force;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat((T) 0.0, x(10), x(11), x(12));

        // X, Y force and Z torque in body frame
        Eigen::Vector<T, 3> rocket_torque;
        rocket_torque << thrust_vector(1) * total_CM,
                -thrust_vector(0) * total_CM,
                (T) 0;

        // -------------- Differential equations ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed variation is Force/mass
        xdot.segment(3, 3) = total_force * mass_inv;

        // Quaternion variation is 0.5*w◦q (if w in inertial frame)
        xdot.segment(6, 4) = (T) 0.5 * (omega_quat * attitude).coeffs();

        // Total torque in body frame
        Eigen::Matrix<T, 3, 1> total_torque;
        total_torque = rocket_torque + body_torque + rot_matrix.transpose() * disturbance_torque;

        // Angular speed variation is Torque/Inertia
        xdot.segment(10, 3) = rot_matrix * (total_torque.cwiseProduct(inertia_inv.template cast<T>()));
    }

};

#endif //SRC_ROCKET_MODEL_HPP
