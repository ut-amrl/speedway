#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "motion/primitives.hpp"

// Object containing all logic and state for racing
class Race {
 public:
  Race();

  void UpdateOdometry(const Eigen::Vector2f& loc, const float angle,
                      const Eigen::Vector2f& vel, const float ang_vel);
  void UpdateLaser(const std::vector<Eigen::Vector2f>& cloud);

  // Performs racing logic and sets desired speed and curvature to execute.
  // Returns success value. A pure function of the current state, as this makes
  // debugging easier later.
  bool Run(float& speed, float& curvature) const;

  motion::LocalPlannerState local_planner_state_;
  std::unique_ptr<motion::SamplerBase> sampler_;
  std::unique_ptr<motion::EvaluatorBase> evaluator_;
  std::unique_ptr<motion::ExecutorBase> executor_;
};
