#include "track/polynomial_regression.h"

#include <glog/logging.h>

#include <eigen3/Eigen/Dense>

namespace track {
PolynomialRegression::PolynomialRegression() {}

void PolynomialRegression::PerformRegression(const std::vector<double>& t,
                                             const std::vector<double>& v,
                                             const uint32_t order) {
  // modified from
  // https://towardsdatascience.com/least-square-polynomial-fitting-using-c-eigen-package-c0673728bd01

  // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order
  // of polynomial, for exame k = 3 for cubic polynomial
  Eigen::MatrixXd T(t.size(), order + 1);
  Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
  Eigen::VectorXd result;

  // check to make sure inputs are correct
  LOG_IF(ERROR, t.size() != v.size())
      << "Different number of inputs (t) and outputs (v): " << t.size()
      << " != " << v.size() << ".";
  LOG_IF(ERROR, t.size() < order + 1)
      << "Not enough inputs to generate polynomial of order " << order
      << ". Requires at least " << order + 1 << ", only " << t.size()
      << " given.";

  // Populate the matrix
  for (size_t i = 0; i < t.size(); ++i) {
    for (size_t j = 0; j < order + 1; ++j) {
      T(i, j) = pow(t.at(i), j);
    }
  }

  // Solve for linear least square fit
  result = T.householderQr().solve(V);
  coeffs_.resize(order + 1);
  for (size_t k = 0; k < order + 1; k++) {
    coeffs_[k] = result[k];
  }
}

double PolynomialRegression::Evaluate(const double t) const {
  double sum = 0;
  for (size_t i = 0; i < coeffs_.size(); i++) {
    sum += coeffs_[i] * pow(t, i);
  }

  return sum;
}
}  // namespace track
