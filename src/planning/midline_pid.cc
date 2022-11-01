#include "midline_pid.h"

#include <eigen3/Eigen/Dense>

#include "track/polynomial_regression.h"

namespace planning {

MidlinePidPlanner::MidlinePidPlanner(double kp, double ki, double kd,
                                     double lookahead)
    : kp_(kp), ki_(ki), kd_(kd), lookahead_(lookahead) {}

double MidlinePidPlanner::Evaluate() {
  double error = ErrorTerm();
  double deriv = last_error_ - error;
  count_++;
  integral_sum_ += error / count_;

  last_error_ = error;

  return kp_ * error + kd_ * deriv + ki_ * integral_sum_;
}

void MidlinePidPlanner::SetPolynomials(const track::Curve& left,
                                       const track::Curve& right) {
  midline_ = track::CreateMidline(left, right);
}

double MidlinePidPlanner::ErrorTerm() {
  lookahead_ = 1;

  Eigen::Vector2f p(midline_.Evaluate(lookahead_));

  return atan2(p.y(), p.x());
}

}  // namespace planning