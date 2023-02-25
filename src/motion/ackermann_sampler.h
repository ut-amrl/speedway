#pragma once

#include "motion/motion_primitives.h"

namespace motion {

class AckermannSampler : public PathSamplerBase {
  void Init(const MotionParameters& params) override;
  std::vector<std::shared_ptr<PathOptionBase>> Sample(
      const int num_samples) override;
};

}  // namespace motion