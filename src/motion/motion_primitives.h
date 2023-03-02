#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "math/line2d.h"
#include "math/poses_2d.h"

namespace motion {

static const float kEpsilon = 1e-6;

class MotionLimits {
 public:
  float max_vel_;
  float max_accel_;
  float max_decel_;
};

class MotionParameters {
 public:
  MotionLimits limits_;

  float robot_length_;
  float robot_width_;
  float base_link_offset_;

  float max_free_path_length_;
  float max_curvature_;
  bool clip_cpoa_;

  float max_clearance_;
  float obstacle_margin_;
  // fraction of the path past which not to consider clearance
  float clearance_clip_fraction_;
};

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

  // Return the pose the robot would be at for fraction f into the path
  // rollout. f \in [0, 1]
  virtual pose_2d::Pose2Df GetIntermediateState(float f) const = 0;

  // Get actuation commands for the robot to execute this rollout in terms of
  // the robot's linear and angular velocity commands for the specified
  // control period.
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

  PathSamplerBase(const MotionParameters& params) { params_ = params; }

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

// enum of different types of evaluators, update whenever you add a new
// evaluator
enum EvaluatorType { LINEAR };

class PathEvaluatorBase {
 public:
  MotionParameters params_;

  Eigen::Vector2f loc_;
  float angle_;
  Eigen::Vector2f vel_;
  Eigen::Vector2f goal_;
  std::vector<Eigen::Vector2f> pointcloud_;

  PathEvaluatorBase(const MotionParameters& params) { params_ = params; }

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

float Run1DTimeOptimalControl(const MotionLimits& limits, const float x_init,
                              const float v_init, const float x_final,
                              const float v_final, const float dt);

float StraightLineClearance(const geometry::Line2f& l,
                            const std::vector<Eigen::Vector2f>& points);

}  // namespace motion