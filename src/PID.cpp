#include "PID.h"
#include <limits>

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

  p_error = numeric_limits<double>::max();
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  if (p_error == numeric_limits<double>::max()) {
    p_error = cte;
  }

  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
