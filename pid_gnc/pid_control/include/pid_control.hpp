#pragma once

#include "rocket_types/rocket_types.h"
#include "rocket_model.hpp"
#include "rocket_types/eigen_conversions.h"
#include "pid_template.hpp"
#include "yaml-cpp/yaml.h"
#include "Eigen/Geometry"
#include "geometry_msgs/Vector3.h"

#include <memory>
#include <vector>
#include <iostream>

using namespace rocket;

#define INNER_ITER 1
#define INNER_INNER_ITER 4
#define INNER_INNER_INNER_ITER 8
#define G 9.8067


class PidController {

public:
    /**
     * @brief Container to manage the a cascade of PID controllers (where each stage itself consists of multiple controllers)
     * 
     */
    struct ControlCascades{
        std::vector<std::vector<PID>> controllers;
        std::vector<std::vector<double>> errors, error_rates, outputs;
        double alpha_limit, beta_limit, dt;
    };

    /**
     * @brief Generate new PidController for use with drone/rocket
     * 
     * @param rocket_props Properties of the targeted vehicle (e.g. mass)
     * @param cc Cascaded PID control loop that has already been initialized (dimensions of vectors, PID weights, limits)
     * @param control_output Reference to variable that carries the ultimative controller output (Gimbal-command)
     * @param u_feed_forward Reference to variable that carries the feedforward term of a higher level controller (e.g. MPC), Note: Usage must be explicitely specified when calling "updateControlLaw()"
     */
    PidController(RocketProps &rocket_props, std::shared_ptr<PidController::ControlCascades>& cc, std::shared_ptr<DroneGimbalControl>& control_output, std::shared_ptr<DroneGimbalControl>& u_feed_forward) :
            rocket_model(rocket_props) {
                this->cc = cc;
                this->u_feed_forward.reset(new rocket::DroneGimbalControl());
                this->control_output.reset(new rocket::DroneGimbalControl());
                this->control_output->inner_angle = 0.0;
                this->control_output->outer_angle = 0.0;
                this->control_output->thrust = 0.0;
                this->control_output->torque = 0.0;
                control_output = this->control_output;
                u_feed_forward = this->u_feed_forward;
                prev_time = 0.0;
                Fz = rocket_model.props.dry_mass * 9.81;
                alpha = 0.0;
                beta = 0.0;
                gamma = 0.0;
    }

    /**
     * @brief Fill given cascaded PID controller container with parameters from yaml file
     * 
     * @param filename Location of yaml file on disk
     * @param cc Reference to cascaded controller
     * @param cascade_names Ordered list of names of cascade stages (same names as in yaml file)
     * @param dimension_names Names of the dimensions in the yaml file (nested dictionary keys)
     * @return true If cc could be configured correctly
     * @return false If an error occured (e.g. specified yaml file not complete)
     */
    static bool loadControllersFromFile(
            std::string filename,
            std::shared_ptr<PidController::ControlCascades>& cc,
            std::vector<std::string> cascade_names,
            std::vector<std::string> dimension_names
        ){
            YAML::Node node;
            try {
                node = YAML::LoadFile(filename);
            }
            catch(std::exception ex){
                std::cout << "Cannot parse YAML file with filename '" << filename << "'\nError message:\n" << ex.what();
            }
            cc->controllers.resize(cascade_names.size());
            cc->errors.resize(cascade_names.size());
            cc->error_rates.resize(cascade_names.size());
            cc->outputs.resize(cascade_names.size());
            // Iterate over the cascade stages
            for(size_t cascade_iter = 0; cascade_iter < cc->controllers.size(); cascade_iter++){
                // Resize the vectors at each stage such that it can store |dimension_names| variables
                cc->errors.at(cascade_iter).resize(dimension_names.size());
                cc->error_rates.at(cascade_iter).resize(dimension_names.size());
                cc->outputs.at(cascade_iter).resize(dimension_names.size());
                cc->controllers.at(cascade_iter).clear();
                // Iterate over the dimensions
                for(size_t dimension_iter = 0; dimension_iter < dimension_names.size(); dimension_iter++){    
                    try
                    {
                        // Create new controller with all gains and limits from file
                        PID ctrl(
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["gains"][0].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["gains"][1].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["gains"][2].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["limits"][0].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["limits"][1].as<double>(),
                            &(cc->errors.at(cascade_iter).at(dimension_iter)),
                            &(cc->error_rates.at(cascade_iter).at(dimension_iter)),
                            &(cc->outputs.at(cascade_iter).at(dimension_iter))
                        );
                        cc->controllers.at(cascade_iter).push_back(ctrl);
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr 
                            << "Error loading element '" 
                            << cascade_names.at(cascade_iter) 
                            << " - " << dimension_names.at(dimension_iter) 
                            << "'. Can not find 3-element-double-list for key 'gains' and 2-element-double-list for key 'limits'.\nError Message:\n" 
                            << e.what() << std::endl;
                        return false;
                    }
                }
            }
            // Fetch limits for maximum attitude set-points (maximum tilting)
            try{
                cc->alpha_limit = node["additional_limits"]["alpha"].as<double>();
                cc->beta_limit = node["additional_limits"]["beta"].as<double>();
            }
            catch(const std::exception& e)
            {
                std::cerr 
                    << "Error loading 'additional_limits'. Can not find double value for either 'alpha' or 'beta' under this key.\nError Message:\n" 
                    << e.what() << std::endl;
                return false;
            }
            return true;  
        }
    
    /**
     * @brief Implementation of a 4-stage cascaded controller (see figure 5.3 of https://drive.google.com/file/d/1psKMbYIDg3n1MyOD7myFiBktjy46THxa/view?usp=sharing)
     * 
     * @param current Current state of the drone/rocket
     * @param set_point Set-Point state of the drone/rocket
     * @param time_s Current time in seconds (required to compute integrative control action)
     * @param angle [optional] Current attitude of vehicle in Euler angles (can be used to debug attitude controller)
     * @param use_u_feed_forward [optional, default=false] Use feedforward control input from higher level controller (e.g. MPC), otherwise: Use gravity compensation as feedforward term
     */
    void updateControlLaw(RocketState current, RocketState set_point, double time_s, geometry_msgs::Vector3* angle, bool use_u_feed_forward = false){
        if(cc->dt < time_s - prev_time) ROS_WARN_STREAM("It has been " << time_s - prev_time << "s since the last update-call. Target is " << cc->dt << "s.");
        else ROS_DEBUG_STREAM("It has been " << time_s - prev_time << "s since the last update-call. Target is " << cc->dt << "s.");
        prev_time = time_s;
        if(inner_inner_inner_iter == INNER_INNER_INNER_ITER){
           // Position control (output: linear velocity set-points)
          inner_inner_inner_iter = 0;
          cc->errors.at(0).at(0) = set_point.position.x - current.position.x;
          cc->errors.at(0).at(1) = set_point.position.y - current.position.y;
          cc->errors.at(0).at(2) = set_point.position.z - current.position.z;
          cc->error_rates.at(0).at(0) = set_point.velocity.x - current.velocity.x;
          cc->error_rates.at(0).at(1) = set_point.velocity.y - current.velocity.y;
          cc->error_rates.at(0).at(2) = set_point.velocity.z - current.velocity.z;
          
          cc->controllers.at(0).at(0).update(cc->dt*INNER_INNER_INNER_ITER);
          cc->controllers.at(0).at(1).update(cc->dt*INNER_INNER_INNER_ITER);
          cc->controllers.at(0).at(2).update(cc->dt*INNER_INNER_INNER_ITER);
        }
        
        if(inner_inner_iter == INNER_INNER_ITER){
            // Linear velocity control (output: linear acceleration set-points)
            inner_inner_iter = 0;
            cc->errors.at(1).at(0) = set_point.velocity.x + cc->outputs.at(0).at(0) - current.velocity.x;
            cc->errors.at(1).at(1) = set_point.velocity.y + cc->outputs.at(0).at(1) - current.velocity.y;
            cc->errors.at(1).at(2) = set_point.velocity.z + cc->outputs.at(0).at(2)- current.velocity.z;
            
            cc->controllers.at(1).at(0).update(cc->dt*INNER_INNER_ITER);
            cc->controllers.at(1).at(1).update(cc->dt*INNER_INNER_ITER);
            cc->controllers.at(1).at(2).update(cc->dt*INNER_INNER_ITER);
            // Compute how much system must be tilted and powered to reach desired accelerations
            convertAccelerationToAngle(rocket_model.props.dry_mass, use_u_feed_forward);
          }        
        if(inner_iter == INNER_ITER){
          // Attitude control with Euler-angles (output: angular rate set-points)
          inner_iter = 0;
          // Compute attitude angles continuously (!) from quaternion
          updateAnglesFromRocketState(current);
          
          
          cc->errors.at(2).at(0) = cc->outputs.at(1).at(0) - alpha;
          cc->errors.at(2).at(1) = cc->outputs.at(1).at(1) - beta;
          cc->errors.at(2).at(2) = -gamma;

          cc->error_rates.at(2).at(0) = -1 * current.angular_velocity.x;
          cc->error_rates.at(2).at(1) = -1 * current.angular_velocity.y;
          cc->error_rates.at(2).at(2) = -1 * current.angular_velocity.z;
          
          cc->controllers.at(2).at(0).update(cc->dt*INNER_ITER);
          cc->controllers.at(2).at(1).update(cc->dt*INNER_ITER);
          cc->controllers.at(2).at(2).update(cc->dt*INNER_ITER);
          
          
        }
        // Attitude rate control (output: Torque set-points)
        cc->errors.at(3).at(0) = set_point.angular_velocity.x + cc->outputs.at(2).at(0) - current.angular_velocity.x;
        cc->errors.at(3).at(1) = set_point.angular_velocity.y + cc->outputs.at(2).at(1) - current.angular_velocity.y;
        cc->errors.at(3).at(2) = set_point.angular_velocity.z + cc->outputs.at(2).at(2) - current.angular_velocity.z;
        
        cc->controllers.at(3).at(0).update(cc->dt, true);
        cc->controllers.at(3).at(1).update(cc->dt, true);
        cc->controllers.at(3).at(2).update(cc->dt, true);
        
        inner_iter++;
        inner_inner_iter++;
        inner_inner_inner_iter++;
        // Convert torque set-points to gimbal-commands
        controlAllocation(use_u_feed_forward);

        // For debug purposes: Quaternion cannot be converted 1:1 to Euler angles due to discontinuities
        // -> We want to know what is actually 'fed' into the attitude controller
        angle->x = alpha * 180. / M_PI;
        angle->y = beta * 180. / M_PI;
        angle->z = gamma * 180. / M_PI;
        
        
        
        
    }
private:
    RocketModel rocket_model;
    std::shared_ptr<PidController::ControlCascades> cc;
    std::shared_ptr<DroneGimbalControl> control_output;
    std::shared_ptr<DroneGimbalControl> u_feed_forward;


    double alpha, beta, gamma, Fz, prev_time;
    int inner_iter = 0;
    int inner_inner_iter = 0;
    int inner_inner_inner_iter = 0;

    /**
     * @brief Given 2 angles (old and new), gets representation for new angle that is the closest to the old one (2-pi "=" 0)
     * 
     * @param newAngle New angle
     * @param prevAngle Old angle
     * @return Representation for new angle that is the closest to the old one
     */
    static double getContinuousAngle(double newAngle, double prevAngle){
        double d = newAngle - prevAngle;
        d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        return prevAngle + d;
      }

    /**
     * @brief Updates class members with now attitude information so that they are continuous
     * 
     * @param s Rocket State that contains an orientation-quaternion
     */
    void updateAnglesFromRocketState(RocketState s){
        Eigen::Matrix<double,3,3> R = Eigen::Quaternion<double>(
            s.orientation.w,
            s.orientation.x,
            s.orientation.y,
            s.orientation.z
            ).toRotationMatrix();
          alpha = getContinuousAngle(atan2(-R(1,2),R(2,2)), alpha);
          beta = getContinuousAngle(atan2(R(0,2),sqrt(R(0,0) * R(0,0) + R(0,1) * R(0,1))), beta);
          gamma = getContinuousAngle(atan2(-R(0,1),R(0,0)), gamma);
    }

    /**
     * @brief Computes required attitude and thrust of drone to obtain desired accelerations (when gimbal angles are 0)
     *        See equations 4.2 to 4.4 of https://drive.google.com/file/d/1psKMbYIDg3n1MyOD7myFiBktjy46THxa/view?usp=sharing
     * @param mass Mass of drone, in kg
     * @param use_mass_feed_forward If mass of drone should be used for feed-forward-style gravity compensation 
     */

    void convertAccelerationToAngle(double mass, bool use_mass_feed_forward){
        
        double tx(mass * cc->outputs.at(1).at(0));
        double ty(mass * cc->outputs.at(1).at(1));
          
        double tz(mass * cc->outputs.at(1).at(2));
        if(!use_mass_feed_forward) tz += mass * G;
        cc->outputs.at(1).at(0) = std::max(std::min(-1 * atan2(ty, tz), cc->alpha_limit), -cc->alpha_limit);
        ty = -1*tan(cc->outputs.at(1).at(0)) * tz;
        cc->outputs.at(1).at(1) = std::max(std::min(cos(cc->outputs.at(1).at(0)) * atan2(tx, tz), cc->beta_limit), -cc->beta_limit);
        tx = tan(cc->outputs.at(1).at(1)) / cos(cc->outputs.at(1).at(0)) * tz;
        Fz = tz / cos(atan2(sqrt(tx*tx+ty*ty), tz));
        
      }
    /**
     * @brief Convert torque-set points to gimbal-control commands
     *        See equations 4.5 to 4.6 of https://drive.google.com/file/d/1psKMbYIDg3n1MyOD7myFiBktjy46THxa/view?usp=sharing
     * @param use_u_feed_forward If feed-forward control-output 'u_feed_forward' should be used (useful especially when tracking other controller)
     */
    void controlAllocation(bool use_u_feed_forward){     
        double Fy = cc->outputs.back().at(0) / rocket_model.props.dry_CM;
        double Fx = -cc->outputs.back().at(1) / rocket_model.props.dry_CM;
        
        double theta_1 = atan2(-Fy,Fz);
        double theta_2 = atan2(cos(theta_1) * Fx, Fz);
        double thrust = std::max(std::min(
          sqrt(Fz * Fz + Fy * Fy + Fx * Fx), (double)rocket_model.props.maxThrust.at(2)
        ), (double)rocket_model.props.minThrust.at(2));

        DroneGimbalControl gimbal_ctrl;
        gimbal_ctrl.inner_angle = theta_2 + (use_u_feed_forward ? this->u_feed_forward->inner_angle : 0);  // Rotation around body-y
        gimbal_ctrl.outer_angle = theta_1 + (use_u_feed_forward ? this->u_feed_forward->outer_angle : 0); // Rotation around body-x
        gimbal_ctrl.thrust = thrust + (use_u_feed_forward ? this->u_feed_forward->thrust : 0);
        gimbal_ctrl.torque = cc->outputs.back().at(2) + (use_u_feed_forward ? this->u_feed_forward->torque : 0);
        *this->control_output = gimbal_ctrl;
  
    } 
};