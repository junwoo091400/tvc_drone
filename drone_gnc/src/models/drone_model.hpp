#ifndef SRC_DRONE_MODEL_HPP
#define SRC_DRONE_MODEL_HPP

#include <Eigen/Eigen>
#include "rocket_model.hpp"

#include <cmath>

class Drone : public Rocket {
public:
    static const int NX = 13;
    static const int NU = 4;
    static const int NP = 10;

    template<typename T>
    using state_t = Eigen::Matrix<T, NX, 1>;
    using state = state_t<double>;
    template<typename T>
    using control_t = Eigen::Matrix<T, NU, 1>;
    using control = control_t<double>;
    template<typename T>
    using parameters_t = Eigen::Matrix<T, NP, 1>;
    using parameters = parameters_t<double>;


    double min_propeller_speed;
    double max_propeller_speed;
    double max_propeller_delta;

    double max_servo1_angle;
    double max_servo2_angle;

    double max_servo_rate;

    double thrust_scaling;
    Eigen::Vector3d disturbance_torque;
    Eigen::Vector3d disturbance_force;

    double torque_scaling;
    double servo1_offset;
    double servo2_offset;

    Drone(ros::NodeHandle &nh) : Rocket(nh) {
        double max_servo1_angle_degree, max_servo2_angle_degree, max_servo_rate_degree;
        if (nh.getParam("/rocket/min_propeller_speed", min_propeller_speed) &&
            nh.getParam("/rocket/max_propeller_speed", max_propeller_speed) &&
            nh.getParam("/rocket/max_propeller_delta", max_propeller_delta) &&
            nh.getParam("/rocket/max_servo1_angle", max_servo1_angle_degree) &&
            nh.getParam("/rocket/max_servo2_angle", max_servo2_angle_degree) &&
            nh.getParam("/rocket/max_servo_rate", max_servo_rate_degree) &&
            nh.getParam("/rocket/CM_to_thrust_distance", total_CM)) {}
        else {
            ROS_ERROR("Failed to get drone parameters");
        }

        max_servo1_angle = max_servo1_angle_degree * (M_PI / 180);
        max_servo2_angle = max_servo2_angle_degree * (M_PI / 180);
        max_servo_rate = max_servo_rate_degree * (M_PI / 180);


        thrust_scaling = 1;
        torque_scaling = 1;
        servo1_offset = 0;
        servo2_offset = 0;
        disturbance_force.setZero();
        disturbance_torque.setZero();
    }

    template<typename T, typename state>
    inline void state_dynamics(const state_t<T> &x,
                               const control_t<T> &u,
                               const parameters_t<T> &params,
                               state &xdot) const {
        Eigen::Matrix<T, 4, 1> input;
        input << u(0), u(1), u(2), u(3);

        T thrust_scaling = params(0);
        T torque_scaling = params(1);
        T servo1_offset = params(2);
        T servo2_offset = params(3);
        Eigen::Matrix<T, 3, 1> dist_force = params.segment(4, 3);
        Eigen::Matrix<T, 3, 1> dist_torque = params.segment(7, 3);

        T servo1 = input(0) + servo1_offset;
        T servo2 = input(1) + servo2_offset;
        T prop_av = input(2);
        T prop_delta = input(3);

        T thrust = getThrust(prop_av) * thrust_scaling;
        T torque = getTorque(prop_delta) * torque_scaling;

        // servomotors thrust vector rotation (see drone_interface for equivalent quaternion implementation)
        Eigen::Matrix<T, 3, 1> thrust_direction;
        thrust_direction << sin(servo2),
                -cos(servo2) * sin(servo1),
                cos(servo1) * cos(servo2);

        Eigen::Matrix<T, 3, 1> thrust_vector = thrust_direction * thrust;

        Eigen::Matrix<T, 3, 1> propeller_torque = thrust_direction * torque;

        Eigen::Matrix<T, 13, 1> x_body = x.segment(0, 13);
        Eigen::Ref<Eigen::Matrix<T, 13, 1>> xdot_body = xdot.segment(0, 13);

        Rocket::generic_rocket_dynamics(x_body, thrust_vector, propeller_torque, dist_force, dist_torque, xdot_body);
    }

    //thrust 2rd order model
    const double a = -0.000035075437403;
    const double b = 0.029719658777622;
    const double c = -0.341510545964088;

    const double g = 9.81;

    // roll model: rot_acc = delta * delta_to_acc
    // torque = I * rot_acc
    const double delta_to_acc = -0.1040;

    double getAverageSpeed(const float thrust) const {
        double average = (-b + std::sqrt(b * b - 4 * a * (c - thrust / g))) / (2 * a);
        return average;
    }

    double getHoverSpeedAverage() {
        return getAverageSpeed(9.81 * dry_mass);
    }


    template<typename T>
    inline T getThrust(const T prop_average) const {
        T thrust = (a * prop_average * prop_average + b * prop_average + c) * g;
        return thrust;
    }

    template<typename T>
    inline T getTorque(const T prop_delta) const {
        T torque = dry_Inertia[2] * prop_delta * delta_to_acc;
        return torque;
    }

    void setParams(double thrust_scaling_val,
                   double torque_scaling_val,
                   double servo1_offset_val, double servo2_offset_val,
                   double fx, double fy, double fz,
                   double mx, double my, double mz) {
        thrust_scaling = thrust_scaling_val;
        torque_scaling = torque_scaling_val;
        servo1_offset = servo1_offset_val;
        servo2_offset = servo2_offset_val;
        disturbance_force << fx, fy, fz;
        disturbance_torque << mx, my, mz;
    }

    template<typename T>
    inline void getParams(parameters_t<T> &params) {
        params << (T) thrust_scaling,
                (T) torque_scaling,
                (T) servo1_offset, (T) servo2_offset,
                (T) disturbance_force.x(), (T) disturbance_force.y(), (T) disturbance_force.z(),
                (T) disturbance_torque.x(), (T) disturbance_torque.y(), (T) disturbance_torque.z();
    }


    void stepRK4(const state x0, const control u, double dT, state &x_next) {
        state k1, k2, k3, k4;

        parameters params;
        getParams(params);

        state_dynamics(x0, u, params, k1);
        state_dynamics((x0 + k1 * dT / 2).eval(), u, params, k2);
        state_dynamics((x0 + k2 * dT / 2).eval(), u, params, k3);
        state_dynamics((x0 + k3 * dT).eval(), u, params, k4);

        x_next = x0 + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    }

};


#endif //SRC_DRONE_MODEL_HPP
