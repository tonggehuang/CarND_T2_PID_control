#include "PID.h"
#include <math.h>
#include <vector>
#include <numeric>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  std::vector<double> kp = {Kp, Ki, Kd};


  cte_sum = 0.0;
  cte_pre = 0.0;
  cte_diff = 0.0;
  Err = 0.0;
  current_step = 0;
  step_limit = 200;
  
  // dpp = 0.2;
  // dpi = 1.0;
  // dpd = 1.0;
  // seq_tol = 0.01;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  cte_sum += cte;
  cte_diff = cte - cte_pre;
  cte_pre = cte;
  steer_value = -Kp*cte - Ki*cte_sum - Kd*cte_diff;

  current_step++;

  if (current_step > step_limit){
    Err += pow(cte, 2);
  }

}

double PID::totalError() {
  if(current_step==0){
    return 1e6;
  }
  return (Err/(current_step - step_limit));
}


double PID::limitSteering(double &steer_value){
  // restrict steer [-1.0, 1.0]
  steer_value = steer_value<-1?-1:steer_value;
  steer_value = steer_value>1?1:steer_value;

  return steer_value;
}

