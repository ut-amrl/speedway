#include "motion/motion_primitives.h"

#include <float.h>

#include <eigen3/Eigen/Dense>

#include "math/line2d.h"
#include "math/math_util.h"

using Eigen::Vector2f;

namespace motion {

float Run1DTimeOptimalControl(const MotionLimits& limits, const float x_init,
                              const float v_init, const float x_final,
                              const float v_final, const float dt) {
  const float dist_left = x_final - x_init;
  const float speed = fabs(v_init);
  const float dv_a = dt * limits.max_accel_;
  const float dv_d = dt * limits.max_decel_;
  float accel_stopping_dist =
      (speed + 0.5 * dv_a) * dt +
      math_util::Sq(speed + dv_a) / (2.0 * limits.max_decel_);
  float cruise_stopping_dist =
      speed * dt + math_util::Sq(speed) / (2.0 * limits.max_decel_);

  if (dist_left > 0) {
    if (speed > limits.max_vel_) {
      return std::max<float>(0.0f, speed - dv_d);
    } else if (speed < limits.max_vel_ && accel_stopping_dist < dist_left) {
      // Acceleration possible.
      return std::min<float>(limits.max_vel_, speed + dv_a);
    } else if (cruise_stopping_dist < dist_left) {
      // Must maintain speed, cruise phase.
      return speed;
    } else {
      // Must decelerate.
      return std::max<float>(0, speed - dv_d);
    }
  } else {
    return std::max<float>(0, speed - dv_d);
  }
}

float StraightLineClearance(const geometry::Line2f& l,
                            const std::vector<Eigen::Vector2f>& points) {
  const Vector2f d = l.Dir();
  const float len = l.Length();
  float clearance = FLT_MAX;
  for (const Vector2f& p : points) {
    const float x = d.dot(p - l.p0);
    if (x < 0.0f || x > len) continue;
    clearance = std::min<float>(clearance, l.Distance(p));
  }
  return clearance;
}

}  // namespace motion
