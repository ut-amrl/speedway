#include "race/race.hpp"

Race::Race() {}

void Race::UpdateOdometry(const Eigen::Vector2f& loc, const float angle) {
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Race::UpdateLaser(const std::vector<Eigen::Vector2f>& cloud) {
  pointcloud_ = cloud;
}

bool Race::Run(float& speed, float& curvature) const {
  speed = 1.0;
  curvature = 0.0;
  return true;
}
