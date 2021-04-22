#ifndef SRC_DRONE_MODEL_HPP
#define SRC_DRONE_MODEL_HPP
#include <Eigen/Eigen>
#include "rocket_model.hpp"

#include <cmath>

class Drone : public Rocket {
public:

    float minPropellerSpeed;
    float maxPropellerSpeed;

    float maxServo1Angle;
    float maxServo2Angle;
    float thrust_scaling;

    void init(ros::NodeHandle &n) {
        Rocket::init(n);
        if (n.getParam("/rocket/minPropellerSpeed", minPropellerSpeed) &&
            n.getParam("/rocket/maxPropellerSpeed", maxPropellerSpeed) &&
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
    }

    template<typename T>
    inline void unScaleControl(Eigen::Matrix<T, 4, 1> &u) const {
        u(0) = maxServo1Angle * u(0);
        u(1) = maxServo2Angle * u(1);
        u(2) = 0.5 * (u(2) + 1) * (maxPropellerSpeed - minPropellerSpeed) + minPropellerSpeed;
        u(3) = 0.5 * (u(3) + 1) * (maxPropellerSpeed - minPropellerSpeed) + minPropellerSpeed;
    }

    void setThrustScaling(double scaling){
        thrust_scaling = scaling;
    }

    template<typename T, typename state>
    inline void state_dynamics(const Eigen::Matrix<T, 13, 1> x,
                               const Eigen::Matrix<T, 4, 1> u,
                               state &xdot) const {
        Eigen::Matrix<T, 4, 1> input;
        input << u(0), u(1), u(2), u(3);

        T servo1 = input(0);
        T servo2 = input(1);
        T bottom = input(2);
        T top = input(3);

        T thrust = getThrust(bottom, top) * thrust_scaling;
        T torque = getTorque(bottom, top);

        // servomotors thrust vector rotation (see drone_interface for equivalent quaternion implementation)
        Eigen::Matrix<T, 3, 1> thrust_direction;
        thrust_direction << cos(servo2) * sin(servo1),
                -sin(servo2),
                cos(servo1) * cos(servo2);

        Eigen::Matrix<T, 3, 1> thrust_vector = thrust_direction * thrust;

        Eigen::Matrix<T, 3, 1> propeller_torque = thrust_direction * torque;

        Eigen::Matrix<T, 13, 1> x_body = x.segment(0, 13);
        Eigen::Ref<Eigen::Matrix<T, 13, 1>> xdot_body = xdot.segment(0, 13);


        Rocket::generic_rocket_dynamics(x_body, thrust_vector, propeller_torque, xdot_body);
    }

    //thrust 3rd order model
    const double a = 1.4902e-04;
    const double b = 0.0143;
    const double c = -0.0140;

    const double g = 9.81;

    // roll model: rot_acc = delta * delta_to_acc
    // torque = I * rot_acc
    const double delta_to_acc = -0.1040;

//    void getPropellersSpeeds(const float thrust, const float torque, float &bottom, float &top) const {
//        float average = (-b + std::sqrt(b * b - 4 * a * (c - thrust/ g))) / (2 * a) ;
//        float delta = torque / (dry_Inertia[2] * delta_to_acc);
//        top = average + delta / 2;
//        bottom = average - delta / 2;
//    }


    template<typename T>
    inline T getThrust(const T bottom, const T top) const {
        T average = (top + bottom) / 2;
        T thrust = (a * average * average + b * average + c) * g;
        return thrust;
    }

    template<typename T>
    inline T getTorque(const T bottom, const T top) const {
        T delta = top - bottom;
        T torque = dry_Inertia[2] * delta * delta_to_acc;
        return torque;
    }


    void stepRK4(const Eigen::Matrix<double, 13, 1> x0, const Eigen::Matrix<double, 4, 1> u, double dT,
             Eigen::Matrix<double, 13, 1> &x_next) {
        Eigen::Matrix<double, 13, 1> k1, k2, k3, k4;

        state_dynamics(x0, u, k1);
        state_dynamics((x0 + k1 * dT / 2).eval(), u, k2);
        state_dynamics((x0 + k2 * dT / 2).eval(), u, k3);
        state_dynamics((x0 + k3 * dT).eval(), u, k4);

        x_next = x0 + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    }

};


#endif //SRC_DRONE_MODEL_HPP