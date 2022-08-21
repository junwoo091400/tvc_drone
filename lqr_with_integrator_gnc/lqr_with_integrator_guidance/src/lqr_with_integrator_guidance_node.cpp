/*
 * Node to send reference trajectory to be tracked by lqr_with_integrator_control node.
 * Also shutdown the engine at the right time to reach apogee altitude
 *
 * Inputs:
 *   - Finite state machine from the lqr_with_integrator_fsm :	    /gnc_fsm_pub
 *   - Estimated state from lqr_with_integrator_navigation:		      /kalman_rocket_state
 *
 * Important parameters:
 *   - Rocket model: 		  /config/rocket_parameters.yaml
 *   - Environment model: 	/config/environment_parameters.yaml
 *
 * Outputs:
 *   - Reference trajectory:                           /target_trajectory
 *   - Zero thrust at engine shutdown time:            /gimbal_command_0
 *
 */

#include "ros/ros.h"

#include "lqr_with_integrator_guidance.hpp"
#include "rocket_utils/Trajectory.h"
#include "rocket_types/ros_conversions.h"

#include <memory>

class RocketGuidanceNode
{
public:
  double frequency = 5;

  RocketGuidanceNode() : rocket_fsm(RocketFSMState::IDLE)
  {
    ros::NodeHandle nh("~");

    std::vector<double> target_apogee_vec = { 0, 0, 0 };
    nh.getParam("/environment/apogee", target_apogee_vec);

    Position target_apogee{ target_apogee_vec[0], target_apogee_vec[1], target_apogee_vec[2] };

    Lqr_with_integratorGuidance::Config guidance_config(target_apogee);

    // Instantiate guidance
    guidance = std::unique_ptr<Lqr_with_integratorGuidance>(new Lqr_with_integratorGuidance(guidance_config));

    // Initialize publishers and subscribers
    initTopics(nh);
  }

  void initTopics(ros::NodeHandle& nh)
  {
    // Create waypoint trajectory publishers
    target_trajectory_pub = nh.advertise<rocket_utils::Trajectory>("/target_trajectory", 10);

    // Subscribe to state message from basic_gnc
    rocket_state_sub = nh.subscribe("/kalman_rocket_state", 1, &RocketGuidanceNode::rocketStateCallback, this);

    // Subscribe to fsm time_keeper
    fsm_sub = nh.subscribe("/gnc_fsm_pub", 1, &RocketGuidanceNode::fsmCallback, this);
  }

  void run()
  {
    double time_now = ros::Time::now().toSec();
    double time_since_launch = time_now - launch_time;

    switch (rocket_fsm)
    {
      case IDLE:
      case RAIL: {
        // Do nothing
        break;
      }

      case LAUNCH: {
        // Update the trajectory
        guidance->computeTrajectory(rocket_state, time_since_launch);

        // Fill the trajectory points' position
        rocket_utils::Trajectory trajectory_msg;
        for (int i = 0; i < n_point_guidance; i++)
        {
          double t = time_since_launch + i * guidance->config.final_time / (n_point_guidance - 1);
          Position position = guidance->sampleTrajectory(t);

          rocket_utils::Waypoint waypoint;
          waypoint.position = toROS(position);
          waypoint.time = time_now + t;

          trajectory_msg.trajectory.push_back(waypoint);
        }

        trajectory_msg.header.stamp = ros::Time(time_now);
        target_trajectory_pub.publish(trajectory_msg);
        break;
      }

      case COAST:
      case STOP: {
        // Do nothing
        break;
      }

      default:
        throw std::runtime_error("Unhandled FSM");
    }
  }

private:
  std::unique_ptr<Lqr_with_integratorGuidance> guidance;

  // Last received rocket state
  RocketState rocket_state{};

  // Last requested fsm
  RocketFSMState rocket_fsm;

  // List of subscribers and publishers
  ros::Publisher target_trajectory_pub;

  ros::Subscriber rocket_state_sub;
  ros::Subscriber fsm_sub;

  double launch_time;
  int n_point_guidance = 10;

  /* ------------ Callbacks functions ------------ */

  // Callback function to store last received fsm
  void fsmCallback(const rocket_utils::FSM::ConstPtr& fsm)
  {
    rocket_fsm = fromROS(*fsm);
    launch_time = fsm->launch_time.toSec();
  }
  // Callback function to store last received state
  void rocketStateCallback(const rocket_utils::State::ConstPtr& rocket_state_msg)
  {
    rocket_state = fromROS(*rocket_state_msg);
  }
};

int main(int argc, char** argv)
{
  // Init ROS guidance node
  ros::init(argc, argv, "guidance");

  RocketGuidanceNode guidance_node;

  ros::Rate loop_rate(guidance_node.frequency);

  while (ros::ok())
  {
    ros::spinOnce();
    guidance_node.run();

    loop_rate.sleep();
  }
}