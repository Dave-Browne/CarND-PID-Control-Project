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
  dp[0] = (Kp==0? 0.1 : Kp/10.);
  dp[1] = (Kd==0? 0.1 : Kd/10.);
  dp[2] = (Ki==0? 0.1 : Ki/10.);
  total_err = 0;
  best_err = 1e5;
  start_threshold = 50;         // start twiddle after threshold
  counter = 0;
  pid_var = 0;      // set twiddle to update P first
  tw_step = 1;      // set twiddle to start at the beginning
  prev_time = clock();
}


void PID::UpdateError(double cte) {
  d_error = (cte - p_error) / delta_t;          // p_error holds the previous cte value
  p_error = cte;
  i_error += cte * delta_t;

  if (counter > start_threshold) total_err += cte*cte;
  counter += 1;                     // increment the number of times car has moved
}


double PID::TotalError() {
  double err = -Kp*p_error - Kd*d_error - Ki*i_error;
  std::cout << "Total_error: " << err << " p_error: " << p_error << " d_error: " << d_error << " i_error: " << i_error << std::endl;
  err = (err > 1? 1:err);
  err = (err < -1? -1:err);
  return err;
}


double PID::IdealSpeed(double steer) {
  int max_speed = 50;
  double x = max_speed - 0.9*max_speed*sqrt(fabs(steer));         // straight = fast, max turn = slow. Non-linear model.
  if (counter < 20) x = counter;                                  // gradual acceleration
  return x;
}

// The aim of twiddle is to update the PID coefficients so that cte is minimised
void PID::Twiddle() {

  if (counter > start_threshold && (fabs(dp[0])+fabs(dp[1])+fabs(dp[2]) > 1e-5)) {

    double curr_err = total_err / (counter - start_threshold);

    // Cycle between P, I and D updates
    if (tw_step == 0) {
//      std::cout << "step 0\n";
      pid_var = (pid_var + 1) % 3;
      tw_step = 1;
    }

    if (tw_step == 1) {
//      std::cout << "step 1\n";
      p[pid_var] += dp[pid_var];
      UpdateParams(p);
      tw_step = 2;
    }
    else if (tw_step == 2) {
//      std::cout << "step 2\n";
      if (curr_err < best_err) {        // CONDITION 1
//        std::cout << "step 2.1\n";
        best_err = curr_err;
        dp[pid_var] *= 1.1;
        tw_step = 0;
        Twiddle();
        return;
      }
      else {
//        std::cout << "step 2.2\n";
        p[pid_var] -= 2*dp[pid_var];
        UpdateParams(p);
        tw_step = 3;
      }
    }
    else if (tw_step == 3) {
//      std::cout << "step 3\n";
      if (curr_err < best_err) {        // CONDITION 2
//        std::cout << "step 3.1\n";
        best_err = curr_err;
        dp[pid_var] *= 1.1;
        tw_step = 0;
        Twiddle();
        return;
      }
      else {                            // CONDITION 3
//        std::cout << "step 3.2\n";
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