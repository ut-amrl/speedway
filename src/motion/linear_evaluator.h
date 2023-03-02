#pragma once

#include "motion/motion_primitives.h"

namespace motion {

class LinearEvaluatorParameters {
 public:
  size_t active_index_ = 0;
  std::vector<float> distance_weights_;
  std::vector<float> clearance_weights_;
  std::vector<float> option_clearance_weights_;
  std::vector<float> free_path_weights_;
  std::vector<float> fpl_avg_windows_;
};

class LinearEvaluator : public PathEvaluatorBase {
 public:
  LinearEvaluatorParameters evaluator_params_;

  LinearEvaluator(const MotionParameters& motion_params,
                  const LinearEvaluatorParameters& evaluator_params);
  std::shared_ptr<PathOptionBase> FindBest(
      const std::vector<std::shared_ptr<PathOptionBase>>& paths) override;

 private:
  float CalculateCost(const float dist_to_goal, const float length,
                      const float option_clearance, const float clearance);
};

}  // namespace motion