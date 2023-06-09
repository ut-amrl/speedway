#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

// Object containing all logic and state for racing
class Race {
 public:
  void UpdateOdometry(const Eigen::Vector2f& loc, const float angle);
  void UpdateLaser(const std::vector<Eigen::Vector2f>& cloud);

  // Performs racing logic and sets desired speed and curvature to execute.
  // Returns success value.
  bool Run(float& speed, float& curvature);
};