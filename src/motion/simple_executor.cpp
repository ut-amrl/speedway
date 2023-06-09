#include "motion/simple_executor.hpp"

namespace motion {
SimpleExecutor::SimpleExecutor() {}

std::pair<float, float> SimpleExecutor::Execute(
    const std::shared_ptr<TrajectoryBase>& trajectory,
    const LocalPlannerState& state) const {
  return {1.0, 0.0};
}
}  // namespace motion
