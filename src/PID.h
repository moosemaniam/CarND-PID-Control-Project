#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
   * Twiddle stuff
   * */
  bool first_run=true;
  double tolerence;
  double twiddle[3];
  double d[3];
  double best_cte;
  int twiddle_index;
  int prev_twiddle_index;
  int update_state;
  double curr_cte;
  double prev_cte;

  double twiddle_sum();
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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte,double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
