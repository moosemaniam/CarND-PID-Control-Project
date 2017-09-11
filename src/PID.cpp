#include "PID.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

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

  prev_cte=0.0;
  curr_cte=0.0f;
  d_error = 0.0f;
  p_error = 0.0f;
  i_error = 0.0f;
  twiddle_index=0;

  tolerence = 0.001;

  best_cte=2048.0f;
  twiddle[0]=Kp;
  twiddle[1]=Kd;
  twiddle[2]=Ki;
  update_state=0;

  twiddle_index=0;
  prev_twiddle_index=0;
  /* Initial delta change to the */
  /*   twiddle */
  d[0]=0.1;
  d[1]=0.1;
  d[2]=0.001;
}

double PID::twiddle_sum()
{
  return(fabs(d[0])+ fabs(d[1]));
}
#define DISABLE_COEF_UPDATE
void PID::UpdateError(double cte,double dt) {

  curr_cte = cte;
  /* The Twiddle tutorial in Sebastian's video can be broken */
  /*   down into the below update state machines */

  //printf("VALUES %f %f %f %f\n",twiddle[0],twiddle[1],d[0],d[1]);
#ifndef DISABLE_COEF_UPDATE
  if twiddle_sum() > tolerence)
  {
    //printf("CTE %f BEST_CTE %f\n",cte,best_cte);
    switch(update_state)
    {
      case 0:
        {

          twiddle[prev_twiddle_index] += d[prev_twiddle_index];
          //printf("case %d:p index %d [%f] d[%f]\n",update_state,prev_twiddle_index,twiddle[prev_twiddle_index],d[prev_twiddle_index]);
          update_state=1;
        }
        break;
      case 1:
        {

          /* Good change. Lets have more of this */
          if(fabs(cte) < fabs(best_cte))
          {
            best_cte = cte;
            d[prev_twiddle_index] *= 1.1;
            //printf("**case %d:p index %d [%f] d[%f]\n",update_state,prev_twiddle_index,twiddle[prev_twiddle_index],d[prev_twiddle_index]);
            update_state=3;
          }
          else
          {
            /* Bad change, revert the change back */
            d[prev_twiddle_index] -= 2* d[prev_twiddle_index];
            //printf("case %d:p index %d [%f] d[%f]\n",update_state,prev_twiddle_index,twiddle[prev_twiddle_index],d[prev_twiddle_index]);
            update_state=2;
          }
        }
        break;
      case 2:
        {
          if(fabs(cte) < fabs(best_cte))
          {
            best_cte = cte;
            d[prev_twiddle_index] *= 1.1;
            //printf("**case %d:p index %d [%f] d[%f]\n",update_state,prev_twiddle_index,twiddle[prev_twiddle_index],d[prev_twiddle_index]);
          }
          else
          {
            twiddle[prev_twiddle_index] += d[prev_twiddle_index];
            d[prev_twiddle_index] *= 0.9;
            //printf("case %d:p index %d [%f] d[%f]\n",update_state,prev_twiddle_index,twiddle[prev_twiddle_index],d[prev_twiddle_index]);
          }
          update_state=3;
        }
        break;
      case 3:
        {
          /* Let move onto the next index to optimize */
          prev_twiddle_index = (prev_twiddle_index + 1) % 2;
          //printf("case %d:p index %d [%f] d[%f]\n",update_state,prev_twiddle_index,twiddle[prev_twiddle_index],d[prev_twiddle_index]);
          update_state = 0;
        }
        break;

    }
  }

  /* Apply Twiddle */
  Kp = twiddle[0];
  Kd = twiddle[1];

  p_error = d[0];
  d_error = d[1];
  i_error = d[2];
#endif
  /* Differential term */
  /* Sum of all previous cross track error over time. */
  i_error +=(curr_cte - prev_cte)* dt;

  /* printf("Kp,Ki,Kd [%f,%f,%f]\n",Kp,Ki,Kd); */
  /* printf("errors [%f,%f,%f]\n\n\n",p_error,d_error,i_error); */

}

double PID::TotalError() {
  double retval = (-Kp * curr_cte - Kd * (curr_cte - prev_cte) );
  /* double retval = (-Kp * p_error - Kd * d_error - Ki * i_error ); */
  //printf("steering calculation cte %f prev_cte %f\n",curr_cte,prev_cte);
  prev_cte = curr_cte;
  return retval;
}

