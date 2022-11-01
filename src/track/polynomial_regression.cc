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

Curve CreateMidline(const Curve& left, const Curve& right) {
  PolynomialRegression mid_x, mid_y;

  // Compute mid_x
  const PolynomialRegression &left_x = left.x, &right_x = right.x;
  std::vector<double> reversed_left_x, reversed_right_x;
  for (int i = (int)left_x.coeffs_.size() - 1; i >= 0; i--) {
    reversed_left_x.push_back(left_x.coeffs_[i]);
  }
  for (int i = (int)right_x.coeffs_.size() - 1; i >= 0; i--) {
    reversed_right_x.push_back(right_x.coeffs_[i]);
  }
  while (reversed_left_x.size() < reversed_right_x.size())
    reversed_left_x.push_back(0);
  while (reversed_right_x.size() < reversed_left_x.size())
    reversed_right_x.push_back(0);
  std::reverse(reversed_left_x.begin(), reversed_left_x.end());
  std::reverse(reversed_right_x.begin(), reversed_right_x.end());

  for (size_t i = 0; i < reversed_left_x.size(); i++) {
    double avg_coeff = (reversed_left_x[i] + reversed_right_x[i]) / 2;
    mid_x.coeffs_.push_back(avg_coeff);
  }

  // Compute mid_y
  const PolynomialRegression &left_y = left.y, &right_y = right.y;
  std::vector<double> reversed_left_y, reversed_right_y;
  for (int i = (int)left_y.coeffs_.size() - 1; i >= 0; i--) {
    reversed_left_y.push_back(left_y.coeffs_[i]);
  }
  for (int i = (int)right_y.coeffs_.size() - 1; i >= 0; i--) {
    reversed_right_y.push_back(right_y.coeffs_[i]);
  }
  while (reversed_left_y.size() < reversed_right_y.size())
    reversed_left_y.push_back(0);
  while (reversed_right_y.size() < reversed_left_y.size())
    reversed_right_y.push_back(0);
  std::reverse(reversed_left_y.begin(), reversed_left_y.end());
  std::reverse(reversed_right_y.begin(), reversed_right_y.end());

  for (size_t i = 0; i < reversed_left_y.size(); i++) {
    double avg_coeff = (reversed_left_y[i] + reversed_right_y[i]) / 2;
    mid_y.coeffs_.push_back(avg_coeff);
  }
  
  return Curve(mid_x, mid_y);
}

Eigen::Vector2f Curve::Evaluate(const double t) const {
  return Eigen::Vector2f(x.Evaluate(t), y.Evaluate(t));
}

}  // namespace track
