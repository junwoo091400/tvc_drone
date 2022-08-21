#pragma once

#include "rocket_types/rocket_types.h"
#include "rocket_model.hpp"
#include "rocket_types/eigen_conversions.h"
#include <memory>
#include "yaml-cpp/yaml.h"
#include "./drone_model.hpp"
#include "mpc_utils.h"
#include "load_drone_props.hpp"
#include "ricatti_solver.hpp"

using namespace rocket;

class LqrController {


public:


    LqrController(DroneProps<double> &drone_props, double dt, std::string QR_file_path):
        dt(dt)
            {
                K.setZero();
                
                
                u_steady_state.setZero();
                state.setZero();
                error.setZero();
                loadQR(QR_file_path);
                drone.init(drone_props);
                updateLQRGain();
    }
    DroneGimbalControl control(RocketState current, RocketState set_point, Eigen::Matrix<double, 4, 1> u_reference){
        this->updateState(current);
        //updateLQRGain(state);
        this->updateError(current, set_point);
        Eigen::Matrix<double, 4, 1> u = K * error + u_reference;

        DroneGimbalControl gimbal_ctrl;
        gimbal_ctrl.inner_angle = u(1);
        gimbal_ctrl.outer_angle = u(0);
        gimbal_ctrl.thrust = u(2);
        gimbal_ctrl.torque = u(3);
        return gimbal_ctrl;
    }
    DroneGimbalControl control(RocketState current, RocketState set_point){
        this->updateState(current);
        //updateLQRGain(state);
        this->updateError(current, set_point);
        Eigen::Matrix<double, 4, 1> u = K * error + u_steady_state;

        DroneGimbalControl gimbal_ctrl;
        gimbal_ctrl.inner_angle = u(1);
        gimbal_ctrl.outer_angle = u(0);
        gimbal_ctrl.thrust = u(2);
        gimbal_ctrl.torque = u(3);
        return gimbal_ctrl;
    }
    bool loadFromFile(std::string K_filename, std::string U_filename){
        YAML::Node node;
        try
        {
            node = YAML::LoadFile(K_filename);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Could not load K-file...\n" << e.what() << '\n';
            return false;
        }
        for(size_t i=0; i < 4; i++) for (size_t j=0; j < 12; j++) K(i,j) = node[i][j].as<double>();
        try
        {
            node = YAML::LoadFile(U_filename);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Could not load us-file...\n" << e.what() << '\n';
            return false;
        }
        for(size_t i=0; i < 4; i++) u_steady_state(i) = node[i].as<double>();
        return true;
        
    }

    


private:
    Eigen::Matrix<double, 4,12> K;
    Matrix<double, 12,12> Q;
    Matrix<double, 4,4> R;
    Eigen::Matrix<double, 4, 1> u_steady_state;
    Eigen::Matrix<double, 12, 1> error, state;
    DroneEuler drone;
    double dt;
    static double getContinuousAngle(double newAngle, double prevAngle){
        double d = newAngle - prevAngle;
        d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        return prevAngle + d;
      }
    
    void loadQR(std::string QR_file_path){
        YAML::Node node;
        try {
            node = YAML::LoadFile(QR_file_path);
        }
        catch(std::exception ex){
            std::cout << "Cannot parse YAML file with filename '" << QR_file_path << "'\nError message:\n" << ex.what();
        }
        Q.setZero();
        for(size_t i=0; i<12; i++) Q(i,i) = node["Q"].as<std::vector<double>>().at(i);
        R.setZero();
        for(size_t i=0; i<4; i++) R(i,i) = node["R"].as<std::vector<double>>().at(i);

        
    }


    void updateState(RocketState current){
        state.segment(9,3) <<   current.angular_velocity.x, current.angular_velocity.y, current.angular_velocity.z;
        state.head(6) <<    
                            current.position.x, current.position.y, current.position.z,current.velocity.x, current.velocity.y, current.velocity.z;
        Eigen::Matrix<double,3,3> R = Eigen::Quaternion<double>(
                    current.orientation.w,
                    current.orientation.x,
                    current.orientation.y,
                    current.orientation.z
                    ).toRotationMatrix();

        state(6) = getContinuousAngle(atan2(-R(1,2),R(2,2)), state(6));
        state(7) = getContinuousAngle(atan2(R(0,2),sqrt(R(0,0) * R(0,0) + R(0,1) * R(0,1))),  state(7));
        state(8) = getContinuousAngle(atan2(-R(0,1),R(0,0)),  state(8));
    }

    void updateError(RocketState current, RocketState set_point){
        error.segment(9,3) <<    set_point.angular_velocity.x - current.angular_velocity.x, set_point.angular_velocity.y - current.angular_velocity.y, set_point.angular_velocity.z - current.angular_velocity.z;
        error.head(6) <<    set_point.position.x-current.position.x, set_point.position.y-current.position.y, set_point.position.z-current.position.z,
                            set_point.velocity.x - current.velocity.x, set_point.velocity.y - current.velocity.y, set_point.velocity.z - current.velocity.z
                            ;
        Eigen::Matrix<double,3,3> R1 = Eigen::Quaternion<double>(
                    current.orientation.w,
                    current.orientation.x,
                    current.orientation.y,
                    current.orientation.z
                    ).toRotationMatrix();
        Eigen::Matrix<double,3,3> R2 = Eigen::Quaternion<double>(
                    set_point.orientation.w,
                    set_point.orientation.x,
                    set_point.orientation.y,
                    set_point.orientation.z
                    ).toRotationMatrix();
        Eigen::Matrix<double,3,3> R = (R2.transpose() * R1).transpose();
        error(6) = getContinuousAngle(atan2(-R(1,2),R(2,2)), error(6));
        error(7) = getContinuousAngle(atan2(R(0,2),sqrt(R(0,0) * R(0,0) + R(0,1) * R(0,1))),  error(7));
        error(8) = getContinuousAngle(atan2(-R(0,1),R(0,0)),  error(8));
    }
    void updateLQRGain(Matrix<double, 12, 1>  x=Matrix<double, 12, 1>::Zero()){
        Matrix<double, 12, 12> A, Phi;
        Matrix<double, 12, 4>  B, Gamma;
        
        // Matrix<double, 12, 1>  x;
        u_steady_state(2) = drone.props.dry_mass * 9.81;
        computeLinearizedModel(&drone,A,B,x,u_steady_state);
        
        discretize<12,4>(A,Phi, B,Gamma, dt);

        // R.diagonal() = Matrix<double, 4, 1>::Ones();
        Matrix<double, 12,12> P;
        solve_riccati_iteration_discrete<double, 12, 4>(Phi, Gamma, Q, R, P);
        this->K = (R+Gamma.transpose() * P * Gamma).inverse() * Gamma.transpose() * P * Phi;
        //std::cout <<"K"<< K(0,0) << std::endl;        
    }

    template<int NX, int NU>
    void discretize(Eigen::Matrix<double, NX, NX> A,Eigen::Matrix<double, NX, NX> &Phi,  Eigen::Matrix<double, NX, NU> B,  Eigen::Matrix<double, NX, NU> &Gamma, double dt){
    Eigen::Matrix<double, 12, 12> A_T = A * dt;
    Phi = A_T.exp();
    Gamma = A.inverse() * (Phi - Eigen::Matrix<double, 12, 12>::Identity()) * B;
    // Only evaluates true if element is NaN (A non-singular and hence inversion not possible)
    if(Gamma(0,0) != Gamma(0,0)){
        // Use common approximation according to: https://en.wikipedia.org/wiki/Discretization#Derivation
        Gamma = Eigen::Matrix<double, 12, 4>::Zero();
        for(size_t i=1; i<1000; i++) Gamma += A.pow(i - 1) * pow(dt, i) * B / tgamma(i+1);
        }
    }

};