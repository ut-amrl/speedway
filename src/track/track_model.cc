#include "track/track_model.h"

#include <glog/logging.h>

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

using Eigen::Vector2f;

namespace track {
TrackModel::TrackModel(const uint32_t polynomial_order,
                       const double wall_tolerance)
    : order_(polynomial_order), wall_tol_(wall_tolerance) {}

void TrackModel::UpdatePointcloud(const std::vector<Vector2f>& cloud,
                                  const float angle_min, const float angle_max,
                                  const float increment) {
  cloud_ = cloud;

  SegmentPointcloud(angle_min, angle_max, increment);
  FitWallPolynomials();
}

std::vector<Vector2f> TrackModel::SampleLeftWall(double interval) const {
  std::vector<Vector2f> points;

  if (left_wall_t_.size() > 0)
    for (double t = 0; t < left_wall_t_.back(); t += interval) {
      points.push_back(
          Vector2f{left_wall_x_pol_.Evaluate(t), left_wall_y_pol_.Evaluate(t)});
    }

  return points;
}

std::vector<Vector2f> TrackModel::SampleRightWall(double interval) const {
  std::vector<Vector2f> points;

  if (right_wall_t_.size() > 0)
    for (double t = 0; t < right_wall_t_.back(); t += interval) {
      points.push_back(Vector2f{right_wall_x_pol_.Evaluate(t),
                                right_wall_y_pol_.Evaluate(t)});
    }

  return points;
}

void TrackModel::SegmentPointcloud(const float angle_min, const float angle_max,
                                   const float increment) {
  // process cloud_ to fill left_wall_points_ and right_wall_points_
  const int left_point_idx = static_cast<int>((M_PI_2 - angle_min) / increment);
  const int right_point_idx =
      static_cast<int>((-M_PI_2 - angle_min) / increment);

  size_t i = right_point_idx;
  double cumulative = 0;

  // move the index to the first point in the right wall
  right_wall_points_.clear();
  right_wall_x_.clear();
  right_wall_y_.clear();
  right_wall_t_.clear();
  while (i != SIZE_MAX && (cloud_[i] - cloud_[i - 1]).norm() < wall_tol_) i--;
  i++;
  while (i < cloud_.size() && (cloud_[i] - cloud_[i - 1]).norm() < wall_tol_) {
    double x = cloud_[i].x();
    double y = cloud_[i].y();

    right_wall_points_.push_back(Vector2f{x, y});
    right_wall_x_.push_back(x);
    right_wall_y_.push_back(y);

    if (right_wall_points_.size() > 1) {
      cumulative += (right_wall_points_.back() -
                     right_wall_points_[right_wall_points_.size() - 2])
                        .norm();
    }
    right_wall_t_.push_back(cumulative);
    i++;
  }

  i = left_point_idx;
  cumulative = 0;
  // move the index to the first point in the left wall
  left_wall_points_.clear();
  left_wall_x_.clear();
  left_wall_y_.clear();
  left_wall_t_.clear();
  while (i < cloud_.size() && (cloud_[i] - cloud_[i - 1]).norm() < wall_tol_)
    i++;
  i--;
  while (i > 0 && (cloud_[i] - cloud_[i - 1]).norm() < wall_tol_) {
    double x = cloud_[i].x();
    double y = cloud_[i].y();

    left_wall_points_.push_back(Vector2f{x, y});
    left_wall_x_.push_back(x);
    left_wall_y_.push_back(y);

    if (left_wall_points_.size() > 1) {
      cumulative += (left_wall_points_.back() -
                     left_wall_points_[left_wall_points_.size() - 2])
                        .norm();
    }
    left_wall_t_.push_back(cumulative);
    i--;
  }
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
  LOG(INFO) << "sizes";
  LOG(INFO) << right_wall_points_.size();
  LOG(INFO) << right_wall_t_.size();
  LOG(INFO) << right_wall_x_.size();
  LOG(INFO) << right_wall_y_.size();

  left_wall_x_pol_.PerformRegression(left_wall_t_, left_wall_x_, order_);
  left_wall_y_pol_.PerformRegression(left_wall_t_, left_wall_y_, order_);

  right_wall_x_pol_.PerformRegression(right_wall_t_, right_wall_x_, order_);
  right_wall_y_pol_.PerformRegression(right_wall_t_, right_wall_y_, order_);
}
}  // namespace track
