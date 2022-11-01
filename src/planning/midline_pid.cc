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

  std::vector<track::Point> points = midline_.sample_along();
  track::Point min_pt = points[0];
  for (int i = 0; i < (int)points.size(); i++) {
    if (std::sqrt(std::pow(points[i].x, 2) + std::pow(points[i].y, 2)) <
        std::sqrt(std::pow(min_pt.x, 2) + std::pow(min_pt.y, 2))) {
      min_pt = points[i];
    }
  }

  Eigen::Vector2f p(midline_.Evaluate(min_pt.t + lookahead_));

  return atan2(p.y(), p.x());
}

}  // namespace planning