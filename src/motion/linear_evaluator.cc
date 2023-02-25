#include "motion/linear_evaluator.h"

// TODO: Implement this

namespace motion {

void LinearEvaluator::Init(const MotionParameters& params) {}
std::shared_ptr<PathOptionBase> LinearEvaluator::FindBest(

    const std::vector<std::shared_ptr<PathOptionBase>>& paths) {
  return nullptr;
}

void LinearEvaluator::UpdateOdometry(const Eigen::Vector2f& odom_loc,
                                     const float odom_angle) {}

void LinearEvaluator::UpdateVelocity(const Eigen::Vector2f& vel) {}

void LinearEvaluator::UpdateGoal(const Eigen::Vector2f& goal) {}

void LinearEvaluator::UpdatePointcloud(
    const std::vector<Eigen::Vector2f>& points) {}

}  // namespace motion
