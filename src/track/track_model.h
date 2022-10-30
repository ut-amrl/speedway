#pragma once

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "track/polynomial_regression.h"

namespace track {

class TrackModel {
 public:
  TrackModel(const uint32_t polynomial_order, const double wall_tolerance);
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud,
                        const float angle_min, const float angle_max,
                        const float increment);

  std::vector<Eigen::Vector2f> SampleLeftWall(double interval = 0.1) const;
  std::vector<Eigen::Vector2f> SampleRightWall(double interval = 0.1) const;

 protected:
  void SegmentPointcloud(const float angle_min, const float angle_max,
                         const float increment);
  void FitWallPolynomials();

  uint32_t order_;
  double wall_tol_;

  std::vector<Eigen::Vector2f> cloud_;
  std::vector<Eigen::Vector2f> left_wall_points_;
  std::vector<Eigen::Vector2f> right_wall_points_;

  std::vector<double> left_wall_t_;
  std::vector<double> left_wall_x_;
  std::vector<double> left_wall_y_;
  PolynomialRegression left_wall_x_pol_;
  PolynomialRegression left_wall_y_pol_;

  std::vector<double> right_wall_t_;
  std::vector<double> right_wall_x_;
  std::vector<double> right_wall_y_;
  PolynomialRegression right_wall_x_pol_;
  PolynomialRegression right_wall_y_pol_;
};
}  // namespace track
