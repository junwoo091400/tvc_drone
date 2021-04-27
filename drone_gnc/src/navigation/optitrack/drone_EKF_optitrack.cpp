#include "drone_EKF_optitrack.h"

void DroneEKF::initEKF(ros::NodeHandle &nh) {
    double x_var, dx_var, z_var, dz_var, att_var, datt_var, thrust_scaling_var, disturbance_torque_var;
    if (nh.getParam("/filter/predict_vars/x", x_var) &&
        nh.getParam("/filter/predict_vars/dz", dx_var) &&
        nh.getParam("/filter/predict_vars/z", z_var) &&
        nh.getParam("/filter/predict_vars/dz", dz_var) &&
        nh.getParam("/filter/predict_vars/att", att_var) &&
        nh.getParam("/filter/predict_vars/datt", datt_var) &&
        nh.getParam("/filter/predict_vars/thrust_scaling", thrust_scaling_var) &&
        nh.getParam("/filter/predict_vars/disturbance_torque", disturbance_torque_var)) {

        std::vector<double> initial_state;
        nh.getParam("initial_state", initial_state);
        state X0(initial_state.data());
        X = X0;

        Q.setZero();
        Q.diagonal() << x_var, x_var, z_var,
                dx_var, dx_var, dz_var,
                att_var, att_var, att_var, att_var,
                datt_var, datt_var, datt_var,
                thrust_scaling_var,
                disturbance_torque_var, disturbance_torque_var, disturbance_torque_var;

        P.setZero();

//        R.setZero();
//        R.diagonal() << accel_x_var, accel_x_var, accel_z_var,
//                gyro_var, gyro_var, gyro_var;
//        // baro_var;
        R.setZero();
        R.diagonal() << 0.01, 0.01, 0.01,
                0.0001, 0.005, 0.005, 0.005;

//        H.setZero();
//        H.block(0, 13, 3, 3).setIdentity(); //accelerometer
//        H.block(3, 10, 3, 3).setIdentity(); //gyro
        // H(6, 2) = 1; //baro

        H.setZero();

        drone.init(nh);

        current_control.servo1 = 0;
        current_control.servo2 = 0;
        current_control.top = 0;
        current_control.bottom = 0;
        received_control = false;

        last_predict_time = ros::Time::now().toSec();

        /** initialize derivatives */
        ADx(X);
        int div_size = ADx.size();
        int derivative_idx = 0;
        for (int i = 0; i < ADx.size(); ++i) {
            ADx(i).derivatives() = state::Unit(div_size, derivative_idx);
            derivative_idx++;
        }
    } else {
        ROS_ERROR("Failed to get kalman filter parameter");
    }

}

void DroneEKF::updateCurrentControl(const drone_gnc::DroneControl::ConstPtr &drone_control) {
    current_control = *drone_control;
    received_control = true;
}

template<typename T>
void DroneEKF::stateDynamics(const state_t<T> &x, state_t<T> &xdot) {
    if(received_control){
        Eigen::Matrix<T, 13, 1> x_drone = x.segment(0, 13);

        Eigen::Matrix<T, 4, 1> u;
        u << current_control.servo1, current_control.servo2, current_control.bottom, current_control.top;

        Eigen::Matrix<T, 4, 1> params = x.segment(13, 4);

        //state derivatives
        drone.state_dynamics(x_drone, u, params, xdot);
    }
    else{
        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

        //state derivatives
        xdot.segment(0, 3) = x.segment(3, 3);
        xdot.segment(3, 3) << 0.0, 0.0, 0.0;
        xdot.segment(6, 4) = 0.5 * (attitude * omega_quat).coeffs();
        xdot.segment(10, 3) << 0.0, 0.0, 0.0;
    }

    // assume parameters unchanged
    xdot.segment(13, 4) << 0.0, 0.0, 0.0, 0.0;
}

template<typename T>
void DroneEKF::measurementModel(const state_t<T> &x, sensor_data_t<T> &z) {
    z.segment(0, 3) = x.segment(0,3);
    z.segment(3, 4) = x.segment(6,4);
}

void DroneEKF::fullDerivative(const state &x, const state_matrix &P, state &xdot, state_matrix &Pdot) {
    //X derivative
    stateDynamics(x, xdot);

    //P derivative
    //propagate xdot autodiff scalar at current x
    ADx = X;
    ad_state Xdot;
    stateDynamics(ADx, Xdot);

    // obtain the jacobian of f(x)
    for (int i = 0; i < Xdot.size(); i++) {
        F.row(i) = Xdot(i).derivatives();
    }

    Pdot = F * P + P * F.transpose() + Q;
}


void DroneEKF::RK4(const state &X, const state_matrix &P, double dT, state &Xnext, state_matrix &Pnext) {
    state k1, k2, k3, k4;
    state_matrix k1_P, k2_P, k3_P, k4_P;

    fullDerivative(X, P, k1, k1_P);
    fullDerivative(X + k1 * dT / 2, P + k1_P * dT / 2, k2, k2_P);
    fullDerivative(X + k2 * dT / 2, P + k2_P * dT / 2, k3, k3_P);
    fullDerivative(X + k3 * dT, P + k3_P * dT, k4, k4_P);

    Xnext = X + (k1 + 2 * k2 + 2 * k3 + k4) * dT / 6;
    Pnext = P + (k1_P + 2 * k2_P + 2 * k3_P + k4_P) * dT / 6;
}


void DroneEKF::predictStep() {
    double dT = ros::Time::now().toSec() - last_predict_time;
    last_predict_time = ros::Time::now().toSec();

    //predict: integrate X and P
    RK4(X, P, dT, X, P);
}

void DroneEKF::updateStep(sensor_data z) {
    //propagate hdot autodiff scalar at current x
    ADx = X;
    ad_sensor_data hdot;
    measurementModel(ADx, hdot);

    // obtain the jacobian of h(x)
    for (int i = 0; i < hdot.size(); i++) {
        H.row(i) = hdot(i).derivatives();
    }

    Matrix<double, NX, NZ> K;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    X = X + K * (z - H * X);
    P = P - K * H * P;
}