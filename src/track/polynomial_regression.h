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

  double max_t;
};

struct Point {
  double t;
  double x, y;
  Point(double t, double x = 0, double y = 0) : t(t), x(x), y(y) {}
};

class Curve {
 public:
  PolynomialRegression x;
  PolynomialRegression y;
  Eigen::Vector2f Evaluate(const double t) const;
  double max_t;
  Curve(PolynomialRegression x, PolynomialRegression y)
      : x(x), y(y), max_t(std::min(x.max_t, y.max_t)) {}
  Curve() {}

  std::vector<Point> sample_along() {
    std::vector<Point> sampled_points;
    double t = 0, step = 0.05;
    while (t < max_t) {
      sampled_points.push_back(Point(t, x.Evaluate(t), y.Evaluate(t)));
      t += step;
    }
    return sampled_points;
  }
};

Curve CreateMidline(const Curve& p1, const Curve& p2);
}  // namespace track
