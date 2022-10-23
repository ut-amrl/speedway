#include "track/track_model.h"

namespace track {
TrackModel::TrackModel() {}

void TrackModel::UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud) {
  cloud_ = cloud;

  SegmentPointcloud();
  FitWallPolynomials();
}

void TrackModel::SegmentPointcloud() {
  // process cloud_ to fill left_wall_points_ and right_wall_points_
}

void TrackModel::FitWallPolynomials() {
  // calculate t values for each wall and fit polynomials to x and y
}

void PolynomialRegression(const std::vector<double>& t,
                          const std::vector<double>& v,
                          std::vector<double>& coeffs, const uint32_t order) {
  // perform least-squares regression of given order to calculate coeffs
}

double EvaluatePolynomial(const double t, const std::vector<double>& coeffs) {
  // evaluate the polynomial given by the coefficients at the given t
  return 0.0;
}

}  // namespace track
