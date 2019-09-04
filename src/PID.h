#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double cte_sum;
  double cte_pre;
  double cte_diff;
  double Err;
  int current_step;
  int step_limit;

  // twiddle parameters
  double p_error;
  double i_error;
  double d_error;

  double dpp;
  double dpi;
  double dpd;

  double steer_value;

  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  void Init(double Kp_, double Ki_, double Kd_);

  void UpdateError(double cte);

  double totalError();

  double limitSteering(double &steer_value);

};

#endif  // PID_H