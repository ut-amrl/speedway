#pragma once

#include "motion/primitives.hpp"

namespace motion {
class SimpleExecutor : public ExecutorBase {
 public:
  SimpleExecutor();
  std::pair<float, float> Execute(
      const std::shared_ptr<TrajectoryBase>& trajectory,
      const LocalPlannerState& state) const override;
};
}  // namespace motion
