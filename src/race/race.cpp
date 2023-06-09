#include "race/race.hpp"

#include <glog/logging.h>

Race::Race(std::string sampler_type, std::string evaluator_type) {
  // TODO: make this part of the type system and do checks in main; i.e. instead
  // of passing strings pass enum
  if (sampler_type == "ackermann") {
    sampler_ =
        std::unique_ptr<motion::SamplerBase>(new motion::AckermannSampler());
  } else {
    LOG(FATAL) << "Unknown sampler type: " << sampler_type;
  }

  if (evaluator_type == "linear") {
    evaluator_ =
        std::unique_ptr<motion::EvaluatorBase>(new motion::LinearEvaluator());
  } else {
    LOG(FATAL) << "Unknown evaluator type: " << evaluator_type;
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
  speed = 1.0;
  curvature = 0.0;
  return true;
}
