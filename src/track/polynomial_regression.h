#pragma once

#include <cstdint>
#include <vector>

namespace track {
class PolynomialRegression {
 public:
  PolynomialRegression();

  void PerformRegression(const std::vector<double>& t,
                         const std::vector<double>& v, const uint32_t order);

  double Evaluate(const double t) const;

 protected:
  std::vector<double> coeffs_;
  uint32_t order_;
};
}  // namespace track
