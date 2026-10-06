#pragma once

#include <algorithm>

// Simple exponential moving-average low-pass filter.
class FirstOrderFilter {
public:
  explicit FirstOrderFilter(double alpha = 1.0) { setAlpha(alpha); }

  double apply(double input) {
    if (!initialized_) {
      state_ = input;
      initialized_ = true;
      return state_;
    }
    state_ += alpha_ * (input - state_);
    return state_;
  }

  void reset(double value = 0.0) {
    state_ = value;
    initialized_ = true;
  }

  void setAlpha(double alpha) { alpha_ = std::clamp(alpha, 0.0, 1.0); }
  double alpha() const { return alpha_; }

private:
  double alpha_{1.0};
  double state_{0.0};
  bool initialized_{false};
};