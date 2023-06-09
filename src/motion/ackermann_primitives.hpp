#pragma once
#include "motion/primitives.hpp"

namespace motion {
class ConstantCurvatureArc : public TrajectoryBase {
 public:
  ConstantCurvatureArc(float curvature, float length);
  float Length() const override;
  Eigen::Vector2f EndPoint() const override;
  float AngularLength() const override;
  Eigen::Vector2f AtT(float t) const override;
  Eigen::Vector2f AtDistance(float s) const override;

 private:
  float curvature_;
  float length_;
};

class AckermannSampler : public SamplerBase {
 public:
  AckermannSampler();
  std::vector<std::shared_ptr<TrajectoryBase>> Sample(
      const LocalPlannerState& state) const override;
};

}  // namespace motion
