#pragma once

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "track/polynomial_regression.h"

namespace track {

class TrackModel {
 public:
  TrackModel();
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud);

  std::vector<Eigen::Vector2f> SampleLeftWall(double interval = 0.1) const;
  std::vector<Eigen::Vector2f> SampleRightWall(double interval = 0.1) const;

 protected:
  void SegmentPointcloud();
  void FitWallPolynomials();

  std::vector<Eigen::Vector2f> cloud_;
  std::vector<Eigen::Vector2f> left_wall_points_;
  std::vector<Eigen::Vector2f> right_wall_points_;

  std::vector<double> left_wall_t_;
  PolynomialRegression left_wall_x_;
  PolynomialRegression left_wall_y_;
  std::vector<double> right_wall_t_;
  PolynomialRegression right_wall_x_;
  PolynomialRegression right_wall_y_;
};
}  // namespace track
