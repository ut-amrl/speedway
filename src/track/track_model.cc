#include "track/track_model.h"

#include <eigen3/Eigen/Dense>
#include <vector>

namespace track {
TrackModel::TrackModel() {}

void TrackModel::UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud) {
  cloud_ = cloud;

  SegmentPointcloud();
  FitWallPolynomials();
}

std::vector<Eigen::Vector2f> TrackModel::SampleLeftWall(double interval) const {
  std::vector<Eigen::Vector2f> points;

  // evaluate the left wall polynomials over a range of t values with the given
  // interval for x and y and return the resultant points

  return points;
}

std::vector<Eigen::Vector2f> TrackModel::SampleRightWall(
    double interval) const {
  std::vector<Eigen::Vector2f> points;

  // evaluate the right wall polynomials over a range of t values with the given
  // interval for x and y and return the resultant points

  return points;
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
