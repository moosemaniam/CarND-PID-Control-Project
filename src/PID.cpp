#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  d_error = 0.0f;
  p_error = 0.0f;
  i_error = 0.0f;
}

void PID::UpdateError(double cte,double dt) {

  /* Differential term */
  d_error = cte - p_error;
  p_error = cte;
  /* Sum of all previous cross track error over time. */
  i_error += (cte * dt);
}

double PID::TotalError() {
  return(-Kp * p_error -Ki * d_error - Ki * i_error );
}

