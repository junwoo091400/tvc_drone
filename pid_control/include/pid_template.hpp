/**
 * @file pid_template.hpp
 * @author Sven Becker (sven.becker@epfl.ch)
 * @brief Basic implementation of general purpos  but output-limited PID with anti-windup on integrator
 * @version 1.0
 * @date 2022-09-04
 *
 * @copyright Copyright (c) 2022
 *
 */

/**
 * @brief Generic PID controller with gains and local integrator state (s.t. anti-windup)
 *
 */
class PID
{
public:
  /**
   * @brief Construct a new PID controller with full configuration (gains) and necessary reference
   *
   * @param P_gain Kp
   * @param I_gain Ki
   * @param D_gain Kd
   * @param lower_limit Output of PID is clipped, integrator state is modified such that this border is not violated by
   * Ki * I
   * @param upper_limit Output of PID is clipped, integrator state is modified such that this border is not violated by
   * Ki * I
   * @param error Variable containing the control error that gets updated by the main program (call-by-reference!)
   * @param error_rate Variable containing the control error rate that gets updated by the main program
   * (call-by-reference!)
   * @param output Variable containing the controller's output that gets computed by calling the 'update()'-method
   * (call-by-reference!)
   */
  PID(double P_gain, double I_gain, double D_gain, double lower_limit, double upper_limit, double* error,
      double* error_rate, double* output)
  {
    this->P_gain = P_gain;
    this->D_gain = D_gain;
    this->I_gain = I_gain;
    this->error = error;
    this->error_rate = error_rate;
    this->output = output;
    this->lower_limit = lower_limit;
    this->upper_limit = upper_limit;
  }
  /**
   * @brief Update the 'output' variable using the content of the error variables and integrator
   *
   * @param dt Time since last call of this method, in seconds
   * @param compute_error_rate_internally If error rate should be computed with the last and current error using
   * division with dt (= first order approximation), or with the error_rate variable
   * @return true If successfull (always the case for now)
   * @return false If failed (never the case for now)
   */
  inline bool update(double dt, bool compute_error_rate_internally = false)
  {
    if (dt <= 0)
      return false;
    this->integrator += *error * dt;
    // anti-windup:
    if (this->integrator * I_gain > upper_limit)
      this->integrator = upper_limit / I_gain;
    if (this->integrator * I_gain < lower_limit)
      this->integrator = lower_limit / I_gain;
    // Use simple derivative approximation if no better information is provided externally; should not be used
    // (approximation formula should be improved)
    if (compute_error_rate_internally)
      *error_rate = (*error - previous_error) / dt;
    else if (this->integrator * I_gain < lower_limit)
      this->integrator = lower_limit / I_gain;
    *this->output =
        std::min(std::max(P_gain * *error + I_gain * integrator + D_gain * (*error_rate), lower_limit), upper_limit);
    this->previous_error = *error;
    return true;
  }

private:
  // Configuration of controller
  double P_gain, D_gain, I_gain, lower_limit, upper_limit;
  // Pointer to error variables
  double *error, *error_rate, *output, previous_error;
  // Integrator's value container
  double integrator = 0;
};