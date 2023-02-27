#include "motion/constant_curvature_arc.h"

#include <eigen3/Eigen/Dense>

#include "math/math_util.h"
#include "math/poses_2d.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;

namespace motion {

ConstantCurvatureArc::ConstantCurvatureArc()
    : curvature_(0), length_(0), angular_length_(0), clearance_(0) {}

ConstantCurvatureArc::ConstantCurvatureArc(const float curvature)
    : curvature_(curvature), length_(0), angular_length_(0), clearance_(0) {}

float ConstantCurvatureArc::Length() const { return length_; }

float ConstantCurvatureArc::AngularLength() const { return angular_length_; }

float ConstantCurvatureArc::Clearance() const { return clearance_; }

bool ConstantCurvatureArc::IsObstacleConstrained() const {
  return obstacle_constrained_;
}

Pose2Df ConstantCurvatureArc::EndPoint() const {
  return GetIntermediateState(1.0);
}

Pose2Df ConstantCurvatureArc::GetIntermediateState(const float f) const {
  if (std::fabs(curvature_) < kEpsilon) {
    return Pose2Df(0, Vector2f(f * length_, 0));
  } else {
    const float a =
        math_util::Sign(curvature_) * length_ * std::fabs(curvature_);
    const float r = 1.0 / curvature_;
    return Pose2Df(a, r * Vector2f(std::sin(a), 1.0 - std::cos(a)));
  }
}

void ConstantCurvatureArc::GetControls(
    const MotionLimits& limits, const float dt, const Vector2f& linear_vel,
    const float angular_vel, Vector2f& vel_cmd, float& ang_vel_cmd) const {
  // TODO: Idea - don't have end velocity be zero for racing purposes?
  vel_cmd.x() =
      Run1DTimeOptimalControl(limits, 0, linear_vel.x(), length_, 0, dt);
  vel_cmd.y() = 0;
  ang_vel_cmd = curvature_ * vel_cmd.x();
}

}  // namespace motion