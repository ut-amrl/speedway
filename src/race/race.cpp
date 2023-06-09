#include "race/race.hpp"

#include <config_reader/config_reader.h>
#include <glog/logging.h>

#include "motion/ackermann_primitives.hpp"
#include "motion/linear_evaluator.hpp"
#include "motion/simple_executor.hpp"

CONFIG_STRING(sampler_type, "RaceParameters.sampler_type");
CONFIG_STRING(evaluator_type, "RaceParameters.evaluator_type");
CONFIG_STRING(executor_type, "RaceParameters.executor_type");

Race::Race() {
  if (CONFIG_sampler_type == "ackermann") {
    sampler_ =
        std::unique_ptr<motion::SamplerBase>(new motion::AckermannSampler());
  } else {
    LOG(FATAL) << "Unknown sampler type: " << CONFIG_sampler_type;
  }

  if (CONFIG_evaluator_type == "linear") {
    evaluator_ =
        std::unique_ptr<motion::EvaluatorBase>(new motion::LinearEvaluator());
  } else {
    LOG(FATAL) << "Unknown evaluator type: " << CONFIG_evaluator_type;
  }

  if (CONFIG_executor_type == "simple") {
    executor_ =
        std::unique_ptr<motion::ExecutorBase>(new motion::SimpleExecutor());
  } else {
    LOG(FATAL) << "Unknown executor type: " << CONFIG_executor_type;
  }
}

void Race::UpdateOdometry(const Eigen::Vector2f& loc, const float angle,
                          const Eigen::Vector2f& vel, const float ang_vel) {
  local_planner_state_.odom_loc = loc;
  local_planner_state_.odom_angle = angle;
  local_planner_state_.odom_vel = vel;
  local_planner_state_.odom_ang_vel = ang_vel;
}

void Race::UpdateLaser(const std::vector<Eigen::Vector2f>& cloud) {
  local_planner_state_.pointcloud = cloud;
}

bool Race::Run(float& speed, float& curvature) const {
  auto samples = sampler_->Sample(local_planner_state_);
  auto best = evaluator_->FindBest(samples, local_planner_state_);
  auto controls = executor_->Execute(best, local_planner_state_);
  speed = controls.first;
  curvature = controls.second;
  return true;
}
