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

  /*
  * Coefficients
  */ 
  double Kp;
  double Kd;
  double Ki;

  double delta_t;
  double prev_time;

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
  * Update the PID coefficients using the twiddle/hill climber algorithm.
  */
  void Twiddle(double cte, double h);

  /*
  * Update PID parameters.
  */
  double UpdateParams(double p[3]);

  /*
  * Calculate the new cross track error with the updated PID coefficients.
  */
  double predictCTE(double cte, double h);
};

#endif /* PID_H */
