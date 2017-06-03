#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0;
  double d_error = 0;
  double i_error = 0;
  double accum_err;
  double best_err;

  /*
  * Coefficients
  */ 
  double Kp;
  double Kd;
  double Ki;
  double p[3];
  double dp[3];
  double sum_dp;

  /*
  * Counters and others
  */ 
  const int training_cycle = 300;   // number of steps for each twiddle run
  const int start_threshold = 100;   // threshold after which twiddle error is calculated
  int counter;
  int tw_step;
  int pid_var;
  bool isInitialised = false;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_val, double Kd_val, double Ki_val);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Define the desired speed.
  */
  double IdealSpeed(double steer);

  /*
  * Update the PID coefficients using the twiddle/hill climber algorithm.
  */
  void Twiddle();

  /*
  * Update PID parameters.
  */
  void UpdateParams(double p[3]);

};

#endif /* PID_H */
