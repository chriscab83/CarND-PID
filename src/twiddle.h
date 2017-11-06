#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"

class Twiddle {
public:
  /*
  * Constructor
  */
  Twiddle(PID& pid);

  /*
  * Destructor
  */
  virtual ~Twiddle();

  /*
  * Update step and error
  */
  void UpdateError(double error);

private:
  // current timestep
  int timestep_;

  // sum of errors
  double err_sum_;

  // pid
  PID& pid_;
}

#endif  // TWIDDLE_H