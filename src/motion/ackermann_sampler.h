#pragma once

#include "motion/constant_curvature_arc.h"
#include "motion/motion_primitives.h"

namespace motion {

class AckermannSampler : public PathSamplerBase {
 public:
  void Init(const MotionParameters& params) override;
  std::vector<std::shared_ptr<PathOptionBase>> Sample(
      const int num_samples) override;

 private:
  void SetMaxPathLength(std::shared_ptr<ConstantCurvatureArc> path);
  void CheckObstacles(std::shared_ptr<ConstantCurvatureArc> path);
};

}  // namespace motion