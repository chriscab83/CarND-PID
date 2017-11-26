#include <iostream>
#include "twiddle.h"

Twiddle::Twiddle(PID& pid, int n, double tol) 
  : state_(State::INIT), n_(n), tol_(tol), timestep_(0), it_(0), 
    err_(0), best_err_(0), pid_(pid), cur_idx_(0) { 
  
  for (size_t i = 0; i < 3; ++i) {
    p_[i] = 0.0;
    dp_[i] = 1.0;
  }

  pid_.Init(p_[0], p_[1], p_[2]);
}

Twiddle::~Twiddle() {

}

void Twiddle::Init(double p[], double dp[]) {
  for (size_t i = 0; i < 3; ++i) {
    p_[i] = p[i];
    dp_[i] = dp[i];
  }

  pid_.Init(p_[0], p_[1], p_[2]);
}

bool Twiddle::Update(double error, std::function<void()> reset) {
  if (state_ == State::FINISHED) {
    // if the twiddle process is finished, skip update.
    return false;
  }

  err_ += (error * error);

  if (++timestep_ >= n_) {
    finalize_pass();

    double sum = 0.0;
    for (size_t i = 0; i < 3; ++i) {
      sum += dp_[i]; 
    }

    if (cur_idx_ == 0 && sum < tol_) {
      state_ = State::FINISHED;
    }

    std::cout << "TWIDDLE STATE:" << std::endl
              << "  Iteration: " << ++it_ << std::endl
              << "  Best_Err:  " << best_err_ << std::endl
              << "  Err:       " << err_ << std::endl
              << "  Cur_Idx:   " << cur_idx_ << std::endl
              << "  P:        " << std::endl
              << "    [0]:     " << p_[0] << std::endl
              << "    [1]:     " << p_[1] << std::endl
              << "    [2]:     " << p_[2] << std::endl
              << "  Dp:        " << std::endl
              << "    [0]:     " << dp_[0] << std::endl
              << "    [1]:     " << dp_[1] << std::endl
              << "    [2]:     " << dp_[2] << std::endl
              << "  Dp_sum:    " << sum << std::endl
              << "  State:     " << static_cast<int>(state_) << std::endl << std::endl;

    err_ = 0;
    timestep_ = 0;
    reset();

    pid_.Init(p_[0], p_[1], p_[2]);

    return true;
  }

  return false;
}

void Twiddle::finalize_pass() {
  err_ = err_ / n_;

  if (state_ == State::INIT) {

    best_err_ = err_;
    state_ = State::FIRST_PASS;

    p_[cur_idx_] += dp_[cur_idx_];

    return;

  }

  if (state_ == State::FIRST_PASS) {

    if (err_ < best_err_) {
      best_err_ = err_;
      dp_[cur_idx_] *= 1.1;

      // move to next coeff
      if (++cur_idx_ >= 3) { cur_idx_ = 0; }
      p_[cur_idx_] += dp_[cur_idx_];
    }
    else {
      p_[cur_idx_] -= 2 * dp_[cur_idx_];
      state_ = State::SECOND_PASS;
    }

    return;
  }

  if (state_ == State::SECOND_PASS) {

    if (err_ < best_err_) {
      best_err_ = err_;
      dp_[cur_idx_] *= 1.1;
    }
    else {
      p_[cur_idx_] += dp_[cur_idx_];
      dp_[cur_idx_] *= 0.9;
    }

    state_ = State::FIRST_PASS;

    // move to next coeff
    if (++cur_idx_ >= 3) { cur_idx_ = 0; }
    p_[cur_idx_] += dp_[cur_idx_];

    return;
  }
}