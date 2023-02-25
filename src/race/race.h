#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "motion/motion_primitives.h"

namespace race {

class RaceParameters {
 public:
  motion::MotionLimits limits_;
  motion::MotionParameters motion_params_;
};

class Race {
 private:
  bool odom_initialized_ = false;

 public:
  RaceParameters params_;

  Eigen::Vector2f position_;
  float angle_;
  Eigen::Vector2f velocity_;
  Eigen::Vector2f goal_;
  std::vector<Eigen::Vector2f> point_cloud_;

  std::unique_ptr<motion::PathSamplerBase> path_sampler_;
  std::unique_ptr<motion::PathEvaluatorBase> path_evaluator_;

  void UpdateOdometry(const Eigen::Vector3f& odom_loc,
                      const Eigen::Quaternionf& odom_quat);
  void UpdateVelocity(const Eigen::Vector2f& vel);
  void UpdateGoal(const Eigen::Vector2f& goal);
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& points);

  void Init(const RaceParameters& params);
  void Run(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
};

}  // namespace race