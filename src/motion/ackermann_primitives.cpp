#include "motion/ackermann_primitives.hpp"

namespace motion {
static const float kEpsilon = 1e-6;

ConstantCurvatureArc::ConstantCurvatureArc(float curvature, float length)
    : curvature_(curvature), length_(length) {}

Eigen::Vector2f ConstantCurvatureArc::EndPoint() const {
  if (fabs(curvature_) < kEpsilon) {
    return {length_, 0.0};
  } else {
    float angle = curvature_ * length_;
    float radius = 1.0 / curvature_;
    return {radius * sin(angle), radius * cos(angle)};
  }
}

float ConstantCurvatureArc::AngularLength() const {
  return curvature_ * length_;
}

Eigen::Vector2f ConstantCurvatureArc::AtT(float t) const {
  return ConstantCurvatureArc(curvature_, length_ * t).EndPoint();
}

Eigen::Vector2f ConstantCurvatureArc::AtDistance(float s) const {
  float t = std::min(std::max(s / length_, 1.0F), 0.0F);
  return AtT(t);
}

AckermannSampler::AckermannSampler() {}

std::vector<std::shared_ptr<TrajectoryBase>> AckermannSampler::Sample(
    const LocalPlannerState& state) const {
  // TODO: implement
  return {};
}
}  // namespace motion
