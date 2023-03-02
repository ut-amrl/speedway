#include "race/race.h"

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "motion/ackermann_sampler.h"
#include "motion/linear_evaluator.h"

namespace race {

void Race::UpdateOdometry(const Eigen::Vector3f& odom_loc,
                          const Eigen::Quaternionf& odom_quat) {
  odom_initialized_ = true;

  position_ = odom_loc.head<2>();
  angle_ = atan2(odom_quat.z(), odom_quat.w());

  path_sampler_->UpdateOdometry(position_, angle_);
  path_evaluator_->UpdateOdometry(position_, angle_);
}

void Race::UpdateVelocity(const Eigen::Vector2f& vel) {
  velocity_ = vel;

  path_sampler_->UpdateVelocity(velocity_);
  path_evaluator_->UpdateVelocity(velocity_);
}

void Race::UpdateGoal(const Eigen::Vector2f& goal) {
  goal_ = goal;

  path_sampler_->UpdateGoal(goal_);
  path_evaluator_->UpdateGoal(goal_);
}

void Race::UpdatePointcloud(const std::vector<Eigen::Vector2f>& points) {
  point_cloud_ = points;

  path_sampler_->UpdatePointcloud(point_cloud_);
  path_evaluator_->UpdatePointcloud(point_cloud_);
}

void Race::Init(const RaceParameters& params) {
  params_ = params;

  path_sampler_ = std::unique_ptr<motion::PathSamplerBase>(
      new motion::AckermannSampler(params_.motion_params_));
  path_evaluator_ =
      std::unique_ptr<motion::PathEvaluatorBase>(new motion::LinearEvaluator(
          params_.motion_params_, params_.linear_evaluator_params_));
}

void Race::Run(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel) {
  if (!odom_initialized_) {
    if (FLAGS_v) LOG(INFO) << "Odometry not initialized";
    return;
  }

  // auto samples = path_sampler_->Sample(params_.num_samples_);
  // auto best_path = path_evaluator_->FindBest(samples);

  cmd_vel = {0, 0};
  cmd_angle_vel = 0;
}

}  // namespace race
