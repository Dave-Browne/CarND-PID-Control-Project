#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp_val, double Kd_val, double Ki_val) {
  p[0] = Kp = Kp_val;
  p[1] = Kd = Kd_val;
  p[2] = Ki = Ki_val;
  accum_err = 0;
  counter = 0;
  if (isInitialised == false) {
    dp[0] = Kp * 10;
    dp[1] = Kd * 10;
    dp[2] = Ki * 10;
    best_err = 1e5;
    pid_var = 0;      // set twiddle to update P first
    tw_step = 1;      // set twiddle to start at the beginning
  }
  isInitialised = true;
}


void PID::UpdateError(double cte) {
  d_error = (cte - p_error);          // p_error holds the previous cte value
  p_error = cte;
  i_error += cte;

  // Update variables for twiddle
  if (counter > start_threshold) accum_err += cte*cte;
  sum_dp = fabs(dp[0]) + fabs(dp[1]) + fabs(dp[2]);
  // Increment the number of times car has moved
  counter += 1;
}


double PID::TotalError() {
  double err = -Kp*p_error - Kd*d_error - Ki*i_error;
  err = (err > 1? 1:err);
  err = (err < -1? -1:err);
  return err;
}


double PID::IdealSpeed(double steer) {
  int max_speed = 50;
  double x = max_speed - 0.7*max_speed*sqrt(fabs(steer));         // straight = fast, max turn = slow. Non-linear model.
  return x;
}

// The aim of twiddle is to update the PID coefficients so that cte is minimised
// Select coefficients and run the simulator for n steps, then check if error has improved.
void PID::Twiddle() {

  double curr_err = accum_err / (counter - start_threshold);

  if (sum_dp > 1e-5) {

    // Cycle between P, I and D updates
    if (tw_step == 0) {
      pid_var = (pid_var + 1) % 3;
      tw_step = 1;
    }

    if (tw_step == 1) {
      p[pid_var] += dp[pid_var];
      UpdateParams(p);
      tw_step = 2;
    }
    else if (tw_step == 2) {
      if (curr_err < best_err) {        // CONDITION 1
        best_err = curr_err;
        dp[pid_var] *= 1.1;
        tw_step = 0;
        Twiddle();
        return;
      }
      else {
        p[pid_var] -= 2*dp[pid_var];
        UpdateParams(p);
        tw_step = 3;
      }
    }
    else if (tw_step == 3) {
      if (curr_err < best_err) {        // CONDITION 2
        best_err = curr_err;
        dp[pid_var] *= 1.1;
        tw_step = 0;
        Twiddle();
        return;
      }
      else {                            // CONDITION 3
        p[pid_var] += dp[pid_var];
        UpdateParams(p);
        dp[pid_var] *= 0.9;
        tw_step = 0;
        Twiddle();
        return;
      }
    }
  }
}


void PID::UpdateParams(double p[3]) {
  Kp = p[0];
  Kd = p[1];
  Ki = p[2];
}