#pragma once

#include <eigen3/Eigen/Dense>

#include "track/polynomial_regression.h"

namespace planning {
class MidlinePidPlanner {
 public:
  MidlinePidPlanner(double kp, double ki, double kd, double lookahead);
  double Evaluate();
  void SetPolynomials(const track::Curve& left, const track::Curve& right);

 protected:
  double ErrorTerm();

  double last_error_;
  double kp_;
  double ki_;
  double kd_;
  double integral_sum_;
  double count_;

  double lookahead_;
  track::Curve midline_;
};
}  // namespace planning