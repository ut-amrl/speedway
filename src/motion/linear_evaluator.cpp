#include "motion/linear_evaluator.hpp"

namespace motion {
LinearEvaluator::LinearEvaluator() {}

float LinearEvaluator::Evaluate(
    const std::shared_ptr<TrajectoryBase>& trajectory,
    const LocalPlannerState& state) {
  // TODO: implement
  return 0.0;
}

std::shared_ptr<TrajectoryBase> LinearEvaluator::FindBest(
    const std::vector<std::shared_ptr<TrajectoryBase>>& trajectories,
    const LocalPlannerState& state) const {
  return nullptr;
}
}  // namespace motion
