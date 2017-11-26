#include <iostream>
#include "PID.h"

using namespace std;

PID::PID() : first_run_(true) { }

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  first_run_ = true;
}

void PID::UpdateError(double cte) {
  if (first_run_) { 
    d_error = 0;
  }
  else {
    d_error = cte - p_error;
  }
  
  i_error += cte;
  p_error = cte;

  first_run_ = false;
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
