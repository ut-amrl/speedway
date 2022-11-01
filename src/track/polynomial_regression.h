#pragma once

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace track {
class PolynomialRegression {
 public:
  PolynomialRegression();

  void PerformRegression(const std::vector<double>& t,
                         const std::vector<double>& v, const uint32_t order);
  double Evaluate(const double t) const;

  std::vector<double> coeffs_;
};

class Curve {
 public:
  PolynomialRegression x;
  PolynomialRegression y;
  Eigen::Vector2f Evaluate(const double t) const;
  Curve(PolynomialRegression x, PolynomialRegression y) : x(x), y(y) {}
  Curve() {}
};

Curve CreateMidline(const Curve& p1, const Curve& p2);
}  // namespace track
