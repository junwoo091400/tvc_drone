#include "drone_EKF.h"

void DroneEKF::init_EKF(ros::NodeHandle &nh) {
    double x_var, dx_var, z_var, dz_var, att_var, datt_var, gyro_var, accel_x_var, accel_z_var, baro_var, thrust_scaling_var;
    if (nh.getParam("/filter/predict_vars/x", x_var) &&
        nh.getParam("/filter/predict_vars/dz", dx_var) &&
        nh.getParam("/filter/predict_vars/z", z_var) &&
        nh.getParam("/filter/predict_vars/dz", dz_var) &&
        nh.getParam("/filter/predict_vars/att", att_var) &&
        nh.getParam("/filter/predict_vars/datt", datt_var) &&
        nh.getParam("/filter/predict_vars/thrust_scaling", thrust_scaling_var) &&
        nh.getParam("/filter/update_vars/gyro", gyro_var) &&
        nh.getParam("/filter/update_vars/accel_x", accel_x_var) &&
        nh.getParam("/filter/update_vars/accel_z", accel_z_var) &&
        nh.getParam("/filter/update_vars/baro", baro_var)) {

        std::vector<double> initial_state;
        nh.getParam("initial_state", initial_state);
        state_t<double> X0(initial_state.data());
        X = X0;

        Q.setZero();
        Q.diagonal() << x_var, x_var, z_var,
                dx_var, dx_var, dz_var,
                att_var, att_var, att_var, att_var,
                datt_var, datt_var, datt_var,
                thrust_scaling_var;

        P.setZero();

        R.setZero();
        R.diagonal() << accel_x_var, accel_x_var, accel_z_var,
                gyro_var, gyro_var, gyro_var;
        // baro_var;
        R_optitrack.setZero();
        R_optitrack.diagonal() << 0.01, 0.01, 0.01,
                0.0001, 0.005, 0.005, 0.005;

        H.setZero();
        H.block(0, 13, 3, 3).setIdentity(); //accelerometer
        H.block(3, 10, 3, 3).setIdentity(); //gyro
        // H(6, 2) = 1; //baro

        H_optitrack.setZero();
        H_optitrack.topLeftCorner(3, 3).setIdentity();
        H_optitrack.block(3, 6, 4, 4).setIdentity();

        drone.init(nh);

        current_control.servo1 = 0;
        current_control.servo2 = 0;
        current_control.top = 0;
        current_control.bottom = 0;
        received_control = false;

        last_predict_time = ros::Time::now().toSec();
    } else {
        ROS_ERROR("Failed to get kalman filter parameter");
    }

}

void DroneEKF::update_current_control(const drone_gnc::DroneControl::ConstPtr &drone_control) {
    current_control = *drone_control;
    received_control = true;
}

template<typename T>
inline void DroneEKF::state_dynamics(const state_t<T> &x, state_t<T> &xdot) {
    // Orientation of the rocket with quaternion
    Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
    attitude.normalize();
    // Angular velocity omega in quaternion format to compute quaternion derivative
    Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

    // -------------- Differential equation ---------------------

    // Position variation is speed
    xdot.segment(0, 3) = x.segment(3, 3);

    double g0 = 9.81;


    // Speed variation is acceleration

    if(received_control){
        double mass = drone.dry_mass;
        T thrust = drone.getThrust(current_control.bottom, current_control.top);
        Eigen::Matrix<T, 3, 1> thrust_direction;
        thrust_direction << cos(current_control.servo2) * sin(current_control.servo1),
                -sin(current_control.servo2),
                cos(current_control.servo1) * cos(current_control.servo2);
        T thrust_scaling = x(13);
        Eigen::Matrix<T, 3, 1> thrust_vector = thrust_direction * thrust * thrust_scaling;
        Eigen::Vector<T, 3> gravity;
        gravity << (T) 0.0, (T) 0.0, g0 * mass;
        Eigen::Vector<T, 3> total_force;
        total_force = attitude._transformVector(thrust_vector) - gravity;

        xdot.segment(3, 3) << total_force / mass;
    }
    else{
        xdot.segment(3, 3) << 0.0, 0.0, 0.0;
    }

    // Quaternion variation is 0.5*w◦q
    xdot.segment(6, 4) = 0.5 * (attitude * omega_quat).coeffs();

    xdot.segment(10, 3) << 0.0, 0.0, 0.0;

    xdot.segment(13, 1) << 0.0;
}

void DroneEKF::RK4(const state_t<double> &X, double dT, state_t<double> &Xnext) {
    state_t<double> k1, k2, k3, k4;

    state_dynamics(X, k1);
    state_dynamics((X + k1 * dT / 2).eval(), k2);
    state_dynamics((X + k2 * dT / 2).eval(), k3);
    state_dynamics((X + k3 * dT).eval(), k4);

    Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
}

void DroneEKF::P_derivative(const state_matrix &P, state_matrix &Pdot) {
    Pdot = F * P + P * F.transpose() + Q;
}

void DroneEKF::RK4_P(const state_matrix &P, double dT, state_matrix &Pnext) {
    state_matrix k1, k2, k3, k4;

    P_derivative(P, k1);
    P_derivative(P + k1 * dT / 2, k2);
    P_derivative(P + k2 * dT / 2, k3);
    P_derivative(P + k3 * dT, k4);

    Pnext = P + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
}

void DroneEKF::predict_step() {
    double dT = ros::Time::now().toSec() - last_predict_time;
    last_predict_time = ros::Time::now().toSec();

    /** initialize derivatives */
    ad_state_t ADx(X);
    int div_size = ADx.size();
    int derivative_idx = 0;
    for (int i = 0; i < ADx.size(); ++i) {
        ADx(i).derivatives() = state_t<double>::Unit(div_size, derivative_idx);
        derivative_idx++;
    }

    //propagate xdot autodiff scalar at current x
    ad_state_t Xdot;
    state_dynamics(ADx, Xdot);

    // obtain the jacobian
    for (int i = 0; i < X.size(); i++) {
        F.row(i) = Xdot(i).derivatives();
    }
    //predict
    RK4(X, dT, X);
    RK4_P(P, dT, P);
}

void DroneEKF::update_step(sensor_t<double> z) {
    Matrix<double, NX, 6> K;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    X = X + K * (z - H * X);
    P = P - K * H * P;

//    X.segment(10, 3) << z.segment(3, 3);
//
//    X.segment(13, 3) << z.segment(0, 3);
}

void DroneEKF::optitrack_update_step(optitrack_sensor_t<double> z) {
    Matrix<double, NX, 7> K;
    K = P * H_optitrack.transpose() * ((H_optitrack * P * H_optitrack.transpose() + R_optitrack).inverse());
    X = X + K * (z - H_optitrack * X);
    P = P - K * H_optitrack * P;
}