#include "planning/midline_pid.h"

#include <glog/logging.h>

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

std::vector<track::Point> MidlinePidPlanner::GetPoints() {
  return midline_.sample_along();
}

double MidlinePidPlanner::ErrorTerm() {
  lookahead_ = 1;

  std::vector<track::Point> points = midline_.sample_along();
  if (points.size() == 0) {
    LOG(ERROR) << "Midline sample returned empty";
    return 0;
  }
  track::Point min_pt = points[0];
  for (int i = 0; i < (int)points.size(); i++) {
    if (std::sqrt(std::pow(points[i].x, 2) + std::pow(points[i].y, 2)) <
        std::sqrt(std::pow(min_pt.x, 2) + std::pow(min_pt.y, 2))) {
      min_pt = points[i];
    }
  }

  Eigen::Vector2f p(
      midline_.Evaluate(std::min(min_pt.t + lookahead_, midline_.max_t)));

  return atan2(p.y(), p.x());
}

}  // namespace planning