#include "motion/ackermann_sampler.h"

// TODO: Implement this

namespace motion {
void AckermannSampler::Init(const MotionParameters& params) {}

std::vector<std::shared_ptr<PathOptionBase>> AckermannSampler::Sample(
    const int num_samples) {
  return {};
}

void AckermannSampler::UpdateOdometry(const Eigen::Vector2f& odom_loc,
                                      const float odom_angle) {}

void AckermannSampler::UpdateVelocity(const Eigen::Vector2f& vel) {}

void AckermannSampler::UpdateGoal(const Eigen::Vector2f& goal) {}

void AckermannSampler::UpdatePointcloud(
    const std::vector<Eigen::Vector2f>& points) {}

}  // namespace motion
