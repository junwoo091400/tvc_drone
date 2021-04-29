#include <ros/ros.h>
#include "drone_gnc/DroneControl.h"
#include "drone_gnc/DroneState.h"
#include "geometry_msgs/Vector3.h"

#include <Eigen/Eigen>
#include "rocket_model.hpp"

#include <cmath>
#include <drone_model.hpp>

class DroneBackupController {
public:
    static const int NX = 12;
    static const int NU = 4;

    using state = Eigen::Matrix<double, NX, 1>;
    using control = Eigen::Matrix<double, NU, 1>;

    Eigen::Matrix<double, NU, NX> K;
    state xs;
    control us;

    std::shared_ptr<Drone> drone;

    DroneBackupController(std::shared_ptr<Drone> drone_ptr) : drone(drone_ptr) {
        K << -0.0063   , 0.0000  , -0.0000  , -0.0419   , 0.0000  ,  0.0000  ,  0.0000,   -2.4747  ,  0.0000    ,0.0000,   -2.4289    ,0.0000,
        0.0000  ,  0.0063   ,0.0000  ,  0.0000  ,  0.0411  ,  0.0000  , -2.3876  ,  0.0000 ,   0.0000,   -2.3020   , 0.0000 ,  -0.0000,
        0.0000 ,  -0.0000 ,  -3.1383  ,  0.0000   ,-0.0000  , -8.2819  ,  0.0000 ,   0.0000  , -0.0000  ,  0.0000    ,0.0000 ,  -0.0000,
        -0.0000  ,  0.0000  ,  0.0000  , -0.0000 ,   0.0000   , 0.0000   ,-0.0000  , -0.0000  ,  9.8985  ,  0.0000   ,-0.0000 ,   9.7559;
        xs.setZero();
        double hover_speed = drone_ptr->getHoverSpeedAverage();
        us << 0, 0, hover_speed, 0;
    }


    drone_gnc::DroneControl getControl(drone_gnc::DroneState &x_msg, geometry_msgs::Vector3 target_apogee) {
        state x;
        x << x_msg.pose.position.x, x_msg.pose.position.y, x_msg.pose.position.z,
                x_msg.twist.linear.x, x_msg.twist.linear.y, x_msg.twist.linear.z,
                x_msg.pose.orientation.x, x_msg.pose.orientation.y, x_msg.pose.orientation.z,
                x_msg.twist.angular.x, x_msg.twist.angular.y, x_msg.twist.angular.z;

        state xs;
        xs << target_apogee.x, target_apogee.y, target_apogee.z,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0;

        control u_tilde = K * (x - xs);
        control u = u_tilde + us;

//        u(0)=0;
//        u(1)=0;
        ROS_ERROR_STREAM(x - xs);
        ROS_ERROR_STREAM(u_tilde);

        drone_gnc::DroneControl drone_control;
        drone_control.servo1 = u(0);
        drone_control.servo2 = u(1);
        drone_control.bottom = u(2) - u(3) / 2;
        drone_control.top = u(2) + u(3) / 2;

        drone_control.servo1 = std::min(std::max(drone_control.servo1, -drone->maxServo1Angle),
                                        drone->maxServo1Angle);
        drone_control.servo2 = std::min(std::max(drone_control.servo2, -drone->maxServo2Angle),
                                        drone->maxServo2Angle);
        drone_control.top = std::min(std::max(drone_control.top, drone->minPropellerSpeed),
                                     drone->maxPropellerSpeed);
        drone_control.bottom = std::min(std::max(drone_control.bottom, drone->minPropellerSpeed),
                                        drone->maxPropellerSpeed);

        return drone_control;
    };
};

