#include "track/track_model.h"

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
  // calculate t values for each wall and fit polynomials to x and y
}
}  // namespace track
