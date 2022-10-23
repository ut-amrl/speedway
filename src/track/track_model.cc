#include "track/track_model.h"

#include <glog/logging.h>

#include <eigen3/Eigen/Dense>
#include <vector>

using Eigen::Vector2f;

namespace track {
TrackModel::TrackModel() {}

void TrackModel::UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud) {
  cloud_ = cloud;

  SegmentPointcloud();
  FitWallPolynomials();
}

std::vector<Vector2f> TrackModel::SampleLeftWall(double interval) const {
  std::vector<Vector2f> points;

  if (left_wall_t_.size() > 0)
    for (double t = 0; t < left_wall_t_.back(); t += interval) {
      points.push_back(
          Vector2f{left_wall_x_.Evaluate(t), left_wall_y_.Evaluate(t)});
    }

  return points;
}

std::vector<Vector2f> TrackModel::SampleRightWall(double interval) const {
  std::vector<Vector2f> points;

  if (right_wall_t_.size() > 0)
    for (double t = 0; t < right_wall_t_.back(); t += interval) {
      points.push_back(
          Vector2f{right_wall_x_.Evaluate(t), right_wall_y_.Evaluate(t)});
    }

  return points;
}

void TrackModel::SegmentPointcloud() {
  // process cloud_ to fill left_wall_points_ and right_wall_points_
}

void TrackModel::FitWallPolynomials() {
  if (left_wall_points_.size() == 0) {
    LOG(WARNING) << "Left wall point cloud is empty.";
    return;
  }
  if (right_wall_points_.size() == 0) {
    LOG(WARNING) << "Right wall point cloud is empty.";
    return;
  }

  // TODO: make it so that t = 0 is for the points directly on either side of
  // the car instead of just the first given point. this makes it so that any
  // extrapolation means a little bit more since both curves will be starting
  // from the same t.

  // calculate t values for each wall
  double cumulative = 0;
  left_wall_t_.clear();
  right_wall_t_.clear();

  left_wall_t_.push_back(0);
  for (size_t i = 1; i < left_wall_points_.size(); i++) {
    cumulative += (left_wall_points_[i] - left_wall_points_[i - 1]).norm();
    left_wall_t_.push_back(cumulative);
  }

  cumulative = 0;
  right_wall_t_.push_back(0);
  for (size_t i = 1; i < right_wall_points_.size(); i++) {
    cumulative += (right_wall_points_[i] - right_wall_points_[i - 1]).norm();
    right_wall_t_.push_back(cumulative);
  }

  // fit polynomials for the x and y of each wall
  std::vector<double> left_vx;
  std::vector<double> left_vy;
  for (size_t i = 1; i < left_wall_points_.size(); i++) {
    cumulative += (left_wall_points_[i] - left_wall_points_[i - 1]).norm();
    left_wall_t_.push_back(cumulative);
  }
  left_wall_x_.PerformRegression(left_wall_t_, left_vx, ORDER);
  left_wall_y_.PerformRegression(left_wall_t_, left_vy, ORDER);

  std::vector<double> right_vx;
  std::vector<double> right_vy;
  for (size_t i = 1; i < right_wall_points_.size(); i++) {
    cumulative += (right_wall_points_[i] - right_wall_points_[i - 1]).norm();
    right_wall_t_.push_back(cumulative);
  }
  right_wall_x_.PerformRegression(right_wall_t_, right_vx, ORDER);
  right_wall_y_.PerformRegression(right_wall_t_, right_vy, ORDER);
}
}  // namespace track
