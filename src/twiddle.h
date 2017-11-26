#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <functional>
#include "PID.h"

class Twiddle {
public:
  enum class State { INIT, FIRST_PASS, SECOND_PASS, FINISHED };

  /*
  * Constructor
  */
  Twiddle(PID& pid, int n = 1000, double tol = 0.2);

  /*
  * Destructor
  */
  virtual ~Twiddle();

  /*
  * Initialize starting and change values
  */
  void Init(double p[], double dp[]);

  /*
  * Update step and error.  Return true if update did cause reset event.
  */
  bool Update(double error, std::function<void()> reset);

private:
  // method to finalize the current pass
  void finalize_pass();

  // Twiddle state
  State state_;

  // total timesteps
  int n_;

  // tolerance
  double tol_;

  // current timestep
  int timestep_;

  // iteration count
  int it_;

  // sum of errors
  double err_;

  // best error
  double best_err_;

  // pid
  PID& pid_;

  // current index
  int cur_idx_;

  // pid values
  double p_[3], dp_[3];
};

#endif  // TWIDDLE_H