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
  Kp = Kp_val;
  Kd = Kd_val;
  Ki = Ki_val;
}

void PID::UpdateError(double cte) {
  d_error = -Kd * (cte - p_error);          // p_error holds the previous cte value
  p_error = -Kp * cte;
  i_error = -Ki * (i_error + cte);
//  std::cout << "p_error: " << p_error << " d_error: " << d_error << " i_error: " << i_error << std::endl;
}

double PID::TotalError() {
  double steer = p_error + d_error + i_error;
  if (steer > 1) steer = 1;
  if (steer < -1) steer = -1;
//  std::cout << "total_error: " << steer << std::endl;
  return steer;
}

// The aim of twiddle is to update the PID coefficients so that cte is minimised
void PID::Twiddle(double cte, double h) {
  // Twiddle variables
  double err;
  double p[3];
  double dp[3];
  p[0] = Kp;
  p[1] = Kd;
  p[2] = Ki;
  dp[0] = dp[1] = dp[2] = 0.01;
  prev_time = clock();

  // The current best_error is the existing cte
  double best_err = cte;

  for (int j=0; j<15; ++j)
  {
    for (int i=0; i<3; ++i)
    {
      p[i] += dp[i];                // increment p[i]
      UpdateParams(p);
      err = predictCTE(cte, h);
      if (fabs(err) < fabs(best_err))       // CONDITION 1
      {
        best_err = err;
        dp[i] *= 1.1;
      }
      else
      {
        p[i] -= 2*dp[i];            // decrement p[i]
        UpdateParams(p);
        err = predictCTE(cte, h);
        if (fabs(err) < fabs(best_err))     // CONDITION 2
        {
          best_err = err;
          dp[i] *= 1.1;
        }
        else                                // CONDITION 3
        {
          p[i] += dp[i];            // return p[i] to original value
          UpdateParams(p);
          dp[i] *= 0.9;
        }
      }
    } // end of for loop
    // Update the cte value for the next loop
    cte = best_err;
    std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << "... dp: " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
    std::cout << "Esmitated CTE: " << cte << std::endl;
  } // end of while loop
} // end of twiddle

double PID::UpdateParams(double p[3]) {
  Kp = p[0];
  Kd = p[1];
  Ki = p[2];
}

// Calculate the new cross track error with the updated PID coefficients
double PID::predictCTE(double cte, double h) {
  UpdateError(cte);
  double steer = TotalError() * 25 * M_PI / 180;                       // angle between =- 25 radians
  double new_cte = cte + h*sin(steer);
  return new_cte;
}