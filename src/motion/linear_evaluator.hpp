#pragma once
#include "motion/primitives.hpp"

namespace motion {
class LinearEvaluator : public EvaluatorBase {
 public:
  LinearEvaluator(float speed_weight, float curvature_weight);
  float Evaluate(const std::shared_ptr<TrajectoryBase>& trajectory,
                 const LocalPlannerState& state) override;
  std::shared_ptr<TrajectoryBase> FindBest(
      const std::vector<std::shared_ptr<TrajectoryBase>>& trajectories,
      const LocalPlannerState& state) const override;
};
}  // namespace motion
