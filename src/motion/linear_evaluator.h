#pragma once

#include "motion/motion_primitives.h"

namespace motion {

class LinearEvaluator : public PathEvaluatorBase {
  void Init(const MotionParameters& params) override;
  std::shared_ptr<PathOptionBase> FindBest(
      const std::vector<std::shared_ptr<PathOptionBase>>& paths) override;
};

}  // namespace motion