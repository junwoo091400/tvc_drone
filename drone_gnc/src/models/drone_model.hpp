#ifndef SRC_DRONE_MODEL_HPP
#define SRC_DRONE_MODEL_HPP

#include <Eigen/Eigen>
#include "rocket_model.hpp"

#include <cmath>

class Drone : public Rocket {
public:
    static const int NX = 13;
    static const int NU = 4;
    static const int NP = 4;

    double minPropellerSpeed;
    double maxPropellerSpeed;
    double maxPropellerDelta;

    double maxServo1Angle;
    double maxServo2Angle;

    double thrust_scaling;
    Eigen::Vector3d disturbance_torque;

    void init(ros::NodeHandle &n) {
        Rocket::init(n);
        if (n.getParam("/rocket/minPropellerSpeed", minPropellerSpeed) &&
            n.getParam("/rocket/maxPropellerSpeed", maxPropellerSpeed) &&
            n.getParam("/rocket/maxPropellerDelta", maxPropellerDelta) &&
            n.getParam("/rocket/maxServo1Angle", maxServo1Angle) &&
            n.getParam("/rocket/maxServo2Angle", maxServo2Angle) &&
            n.getParam("/rocket/CM_to_thrust_distance", total_CM)) {}
        else {
            ROS_ERROR("Failed to get drone parameters");
        }

        J_inv[0] = 1 / dry_Inertia[0];
        J_inv[1] = 1 / dry_Inertia[1];
        J_inv[2] = 1 / dry_Inertia[2];

        thrust_scaling = 1;
        disturbance_torque.setZero();
    }

    template<typename T>
    inline void unScaleControl(Eigen::Matrix<T, 4, 1> &u) const {
        u(0) = maxServo1Angle * u(0);
        u(1) = maxServo2Angle * u(1);
        u(2) = 0.5 * (u(2) + 1) * (maxPropellerSpeed - minPropellerSpeed) + minPropellerSpeed;
        u(3) = u(3) * maxPropellerDelta;
    }


    inline void scaleControl(Eigen::Matrix<double, 4, 1> &u) const {
        u(0) = u(0) / maxServo1Angle;
        u(1) = u(1) / maxServo2Angle;
        u(2) = 2 * (u(2) - minPropellerSpeed) / (maxPropellerSpeed - minPropellerSpeed) - 1;
        u(3) = u(3) / maxPropellerDelta;
    }


    template<typename T, typename state>
    inline void state_dynamics(const Eigen::Matrix<T, NX, 1> &x,
                               const Eigen::Matrix<T, NU, 1> &u,
                               const Eigen::Matrix<T, NP, 1> &params,
                               state &xdot) const {
        Eigen::Matrix<T, 4, 1> input;
        input << u(0), u(1), u(2), u(3);

        T servo1 = input(0);
        T servo2 = input(1);
        T prop_av = input(2);
        T prop_delta = input(3);

        T thrust_scaling = params(0);
        Eigen::Matrix<T, 3, 1> dist_torque = params.segment(1, 3);

        T thrust = getThrust(prop_av) * thrust_scaling;
        T torque = getTorque(prop_delta);

        // servomotors thrust vector rotation (see drone_interface for equivalent quaternion implementation)
        Eigen::Matrix<T, 3, 1> thrust_direction;
        thrust_direction << cos(servo2) * sin(servo1),
                -sin(servo2),
                cos(servo1) * cos(servo2);

        Eigen::Matrix<T, 3, 1> thrust_vector = thrust_direction * thrust;

        Eigen::Matrix<T, 3, 1> propeller_torque = thrust_direction * torque;

        Eigen::Matrix<T, 13, 1> x_body = x.segment(0, 13);
        Eigen::Ref<Eigen::Matrix<T, 13, 1>> xdot_body = xdot.segment(0, 13);

        Rocket::generic_rocket_dynamics(x_body, thrust_vector, (propeller_torque + dist_torque).eval(), xdot_body);
    }

    //thrust 3rd order model
    const double a = 1.4902e-04;
    const double b = 0.0143;
    const double c = -0.0140;

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

    void setParams(double thrust_scaling_val, double tx, double ty, double tz) {
        thrust_scaling = thrust_scaling_val;
        disturbance_torque << tx, ty, tz;
    }

    template<typename T>
    inline void getParams(Eigen::Matrix<T, 4, 1> &params) {
        params
                << (T) thrust_scaling, (T) disturbance_torque.x(), (T) disturbance_torque.y(), (T) disturbance_torque.z();
    }


    void stepRK4(const Eigen::Matrix<double, NX, 1> x0, const Eigen::Matrix<double, NU, 1> u, double dT,
                 Eigen::Matrix<double, NX, 1> &x_next) {
        Eigen::Matrix<double, NX, 1> k1, k2, k3, k4;

        Eigen::Matrix<double, NU, 1> params;
        getParams(params);

        state_dynamics(x0, u, params, k1);
        //TODO
        k1.segment(3, 3) = 1e-2 * k1.segment(3, 3);
        state_dynamics((x0 + k1 * dT / 2).eval(), u, params, k2);
        //TODO
        k2.segment(3, 3) = 1e-2 * k2.segment(3, 3);
        state_dynamics((x0 + k2 * dT / 2).eval(), u, params, k3);
        //TODO
        k3.segment(3, 3) = 1e-2 * k3.segment(3, 3);
        state_dynamics((x0 + k3 * dT).eval(), u, params, k4);
        //TODO
        k4.segment(3, 3) = 1e-2 * k4.segment(3, 3);

        x_next = x0 + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    }

};


#endif //SRC_DRONE_MODEL_HPP
