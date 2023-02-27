#pragma once

#include "motion/motion_primitives.h"

namespace motion {

class ConstantCurvatureArc : PathOptionBase {
 public:
  float curvature_;
  float length_;
  float angular_length_;
  float clearance_;
  bool obstacle_constrained_;

  ConstantCurvatureArc();
  ConstantCurvatureArc(const float curvature);

  float Length() const override;
  float AngularLength() const override;
  float Clearance() const override;
  bool IsObstacleConstrained() const override;
  pose_2d::Pose2Df EndPoint() const override;
  pose_2d::Pose2Df GetIntermediateState(const float f) const override;
  void GetControls(const MotionLimits& limits, const float dt,
                   const Eigen::Vector2f& linear_vel, const float angular_vel,
                   Eigen::Vector2f& vel_cmd, float& ang_vel_cmd) const override;
};

}  // namespace motion