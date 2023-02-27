#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "math/poses_2d.h"

namespace motion {

static const float kEpsilon = 1e-6;

class MotionLimits {
 public:
  float max_vel_;
  float max_accel_;
  float max_decel_;

  MotionLimits() : max_vel_(0), max_accel_(0), max_decel_(0) {}
  MotionLimits(float max_vel, float max_accel, float max_decel)
      : max_vel_(max_vel), max_accel_(max_accel), max_decel_(max_decel) {}
};

class MotionParameters {};

class PathOptionBase {
 public:
  // Free path length of the rollout -- this is the cumulative free space
  // distance along the direction of the rollout along the path, $\int
  // ||v(t)||dt$ where $v(t)$ is the instantaneous velocity.
  virtual float Length() const = 0;

  // Angular Length of the path rollout -- this is cumulative angular distance
  // (not displacement) traversed: $\int ||\dot{\theta}(t)||dt$ where
  // $\dot{\theta}(t)$ is the instantaneous angular velocity.
  virtual float AngularLength() const = 0;

  // The obstacle clearance along the path.
  virtual float Clearance() const = 0;

  // Indicates whether the path rollout terminates at an obstacle.
  virtual bool IsObstacleConstrained() const = 0;

  // The pose of the robot at the end of the path rollout.
  virtual pose_2d::Pose2Df EndPoint() const = 0;

  // Return the pose the robot would be at for fraction f into the path rollout.
  // f \in [0, 1]
  virtual pose_2d::Pose2Df GetIntermediateState(float f) const = 0;

  // Get actuation commands for the robot to execute this rollout in terms of
  // the robot's linear and angular velocity commands for the specified control
  // period.
  virtual void GetControls(const MotionLimits& limits, const float dt,
                           const Eigen::Vector2f& linear_vel,
                           const float angular_vel, Eigen::Vector2f& vel_cmd,
                           float& ang_vel_cmd) const = 0;
};

class PathSamplerBase {
 public:
  MotionParameters params_;

  Eigen::Vector2f loc_;
  float angle_;
  Eigen::Vector2f vel_;
  Eigen::Vector2f goal_;
  std::vector<Eigen::Vector2f> pointcloud_;

  virtual void Init(const MotionParameters& params) = 0;
  virtual std::vector<std::shared_ptr<PathOptionBase>> Sample(
      const int num_samples) = 0;

  virtual void UpdateOdometry(const Eigen::Vector2f& odom_loc,
                              const float odom_angle) {
    loc_ = odom_loc;
    angle_ = odom_angle;
  }
  virtual void UpdateVelocity(const Eigen::Vector2f& vel) { vel_ = vel; }
  virtual void UpdateGoal(const Eigen::Vector2f& goal) { goal_ = goal; }
  virtual void UpdatePointcloud(const std::vector<Eigen::Vector2f>& points) {
    pointcloud_ = points;
  }
};

class PathEvaluatorBase {
 public:
  MotionParameters params_;

  Eigen::Vector2f loc_;
  float angle_;
  Eigen::Vector2f vel_;
  Eigen::Vector2f goal_;
  std::vector<Eigen::Vector2f> pointcloud_;

  virtual void Init(const MotionParameters& params) = 0;
  virtual std::shared_ptr<PathOptionBase> FindBest(
      const std::vector<std::shared_ptr<PathOptionBase>>& paths) = 0;

  virtual void UpdateOdometry(const Eigen::Vector2f& odom_loc,
                              const float odom_angle) {
    loc_ = odom_loc;
    angle_ = odom_angle;
  }
  virtual void UpdateVelocity(const Eigen::Vector2f& vel) { vel_ = vel; }
  virtual void UpdateGoal(const Eigen::Vector2f& goal) { goal_ = goal; }
  virtual void UpdatePointcloud(const std::vector<Eigen::Vector2f>& points) {
    pointcloud_ = points;
  }
};

float Run1DTimeOptimalControl(const MotionLimits& limits,
                              const float x_init,
                              const float v_init,
                              const float x_final,
                              const float v_final,
                              const float dt);

}  // namespace motion