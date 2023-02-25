#pragma once

#include "motion/motion_primitives.h"

namespace motion {

class AckermannSampler : public PathSamplerBase {
  void Init(const MotionParameters& params) override;
  std::vector<std::shared_ptr<PathOptionBase>> Sample(
      const int num_samples) override;

  void UpdateOdometry(const Eigen::Vector2f& odom_loc,
                      const float odom_angle) override;
  void UpdateVelocity(const Eigen::Vector2f& vel) override;
  void UpdateGoal(const Eigen::Vector2f& goal) override;
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& points) override;
};

}  // namespace motion