#include "twiddle.h"

Twiddle::Twiddle(PID& pid) : _timestep(0) { 
  this->pid_ = pid;
}

Twiddle::~Twiddle() { 

}

void Twiddle::UpdateError(double error) {
  sum_err_ += error;
  ++timestemp_;
}