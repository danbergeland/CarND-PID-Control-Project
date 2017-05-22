#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  
  /*
  * Errors
  */
  p_error=0;
  i_error=0;
  d_error=0;

  cte_last = 0;
  cte_integral = 0;
  /*
  * Coefficients
  */ 
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
}

void PID::UpdateError(double cte) {
  double cte_diff = cte - cte_last;
  
  p_error = Kp*cte;
  i_error = Ki*cte_integral;
  d_error = Kd*cte_diff;
  
  cte_last = cte;
  cte_integral += cte;
}

double PID::TotalError() {
  return (-1*(p_error+i_error+d_error));
}




