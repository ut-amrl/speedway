#pragma once

#include "motion/motion_primitives.h"

namespace motion {

class LinearEvaluator : public PathEvaluatorBase {
  void Init(const MotionParameters& params) override;
  std::shared_ptr<PathOptionBase> FindBest(
      const std::vector<std::shared_ptr<PathOptionBase>>& paths) override;

  void UpdateOdometry(const Eigen::Vector2f& odom_loc,
                      const float odom_angle) override;
  void UpdateVelocity(const Eigen::Vector2f& vel) override;
  void UpdateGoal(const Eigen::Vector2f& goal) override;
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& points) override;
};

}  // namespace motion