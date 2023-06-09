#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

// Object containing all logic and state for racing
class Race {
 public:
  void UpdateOdometry(const Eigen::Vector2f& loc, const float angle);
  void UpdateLaser(const std::vector<Eigen::Vector2f>& cloud);

  // Performs racing logic and sets desired speed and curvature to execute.
  // Returns success value. A pure function of the current state, as this makes
  // debugging easier later.
  bool Run(float& speed, float& curvature) const;

  // Location of robot in odom frame
  Eigen::Vector2f odom_loc_;
  // Angle of robot in odom frame
  float odom_angle_;

  // Raw pointcloud in base_link frame
  std::vector<Eigen::Vector2f> pointcloud_;
};
