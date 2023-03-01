#include "motion/ackermann_sampler.h"

#include <cmath>
#include <eigen3/Eigen/Dense>

#include "config_reader/config_reader.h"
#include "math/math_util.h"
#include "motion/constant_curvature_arc.h"

using Eigen::Vector2f;

namespace motion {

void AckermannSampler::Init(const MotionParameters& params) {
  params_ = params;
}

std::vector<std::shared_ptr<PathOptionBase>> AckermannSampler::Sample(
    const int num_samples) {
  const float c_min = -params_.max_curvature_;
  const float c_max = params_.max_curvature_;
  const float dc = (c_max - c_min) / num_samples;

  std::vector<std::shared_ptr<PathOptionBase>> samples;
  for (int i = 0; i < num_samples; ++i) {
    const float c = c_min + i * dc;
    std::shared_ptr<ConstantCurvatureArc> sample(new ConstantCurvatureArc(c));
    SetMaxPathLength(sample);
    CheckObstacles(sample);
    samples.push_back(sample);
  }

  return samples;
}

void AckermannSampler::SetMaxPathLength(
    std::shared_ptr<ConstantCurvatureArc> path) {
  // TODO: change this if we change the final velocity for TOC
  const float stopping_dist =
      math_util::Sq(vel_.x()) / (2.0 * params_.limits_.max_decel_);
  if (params_.clip_cpoa_) {
    if (std::fabs(path->curvature_) < kEpsilon) {
      const float cpoa_length =
          std::min(params_.max_free_path_length_, goal_.x());
      path->length_ = std::max(cpoa_length, stopping_dist);
    } else {
      const float turn_radius = 1.0f / path->curvature_;
      const float quarter_circle_dist = std::fabs(turn_radius) * M_PI_2;
      const Vector2f turn_center(0, turn_radius);
      const Vector2f target_radial = goal_ - turn_center;
      const Vector2f middle_radial =
          std::fabs(turn_radius) * target_radial.normalized();
      const float middle_angle = std::atan2(std::fabs(middle_radial.x()),
                                            std::fabs(middle_radial.y()));
      const float dist_cpoa = middle_angle * std::fabs(turn_radius);
      const float cpoa_length = std::min<float>(
          {params_.max_free_path_length_, quarter_circle_dist, dist_cpoa});
      path->length_ = std::max(cpoa_length, stopping_dist);
    }
  } else {
    path->length_ = std::max(stopping_dist, params_.max_free_path_length_);
    if (std::fabs(path->curvature_) > kEpsilon) {
      path->length_ = std::min<float>(path->length_,
                                      1.5 * M_PI / std::fabs(path->curvature_));
    }
  }
}

void AckermannSampler::CheckObstacles(
    std::shared_ptr<ConstantCurvatureArc> path) {
  path->obstacle_constrained_ = false;
  path->clearance_ = params_.max_clearance_;
  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * params_.robot_length_ - params_.base_link_offset_ +
                  params_.obstacle_margin_;
  // How much the robot's body extends behind its base link frame.
  const float l_back = 0.5 * params_.robot_length_ + params_.base_link_offset_ +
                       params_.obstacle_margin_;
  // The robot's half-width.
  const float w = 0.5 * params_.robot_width_ + params_.obstacle_margin_;
  const float stopping_dist =
      vel_.squaredNorm() / (2.0 * params_.limits_.max_decel_);
  if (std::fabs(path->curvature_) < kEpsilon) {
    for (const Vector2f& p : pointcloud_) {
      if (std::fabs(p.y()) > w || p.x() < -l_back) continue;
      if (path->length_ > p.x() - l) {
        path->obstacle_constrained_ = true;
        path->length_ = p.x() - l;
        path->obstruction_ = p;
      }
    }
    for (const Vector2f& p : pointcloud_) {
      if (p.x() > params_.clearance_clip_fraction_ * path->length_ + l ||
          p.x() < -l_back)
        continue;  // the point is outside the clearance area
      path->clearance_ = std::min<float>(path->clearance_, std::fabs(p.y()));
    }
    path->clearance_ = std::max(0.0f, path->clearance_);
    path->length_ = std::max(0.0f, path->length_);
    if (path->length_ < stopping_dist) {
      path->length_ = 0;
    }
  } else {
    const float radius = 1.0 / path->curvature_;
    const Vector2f c(0, radius);
    const float s = ((radius > 0.0) ? 1.0 : -1.0);
    const Vector2f inner_front_corner(l, s * w);
    const Vector2f outer_front_corner(l, -s * w);
    const float r1 = std::max<float>(0.0f, std::fabs(radius) - w);
    const float r1_sq = math_util::Sq(r1);
    const float r2_sq = (inner_front_corner - c).squaredNorm();
    const float r3_sq = (outer_front_corner - c).squaredNorm();
    float angle_min = M_PI;
    path->obstruction_ = Vector2f(-params_.max_free_path_length_, 0);
    // The x-coordinate of the rear margin.
    const float x_min = -0.5 * params_.robot_length_ +
                        params_.base_link_offset_ - params_.obstacle_margin_;

    using std::isfinite;
    for (const Vector2f& p : pointcloud_) {
      if (!isfinite(p.x()) || !isfinite(p.y()) || p.x() < 0.0f)
        continue;  // the point is behind the car or invalid
      if (p.x() > x_min && p.x() < l && std::fabs(p.y()) < w) {
        // TODO: maybe remove this check? might be useful for not stopping post
        // crash
        // the point is within the robot plus margin boundary
        path->length_ = 0;
        path->obstruction_ = p;
        angle_min = 0;
        break;
      }

      const float r_sq = (p - c).squaredNorm();
      if (r_sq < r1_sq || r_sq > r3_sq)
        continue;  // the point is outside the swept area
      const float r = std::sqrt(r_sq);
      const float theta = path->curvature_ > kEpsilon
                              ? std::atan2<float>(p.x(), radius - p.y())
                              : std::atan2<float>(p.x(), p.y() - radius);
      float alpha;
      if (r_sq < r2_sq) {
        // Point will hit the side of the robot first.
        const float x = fabs(radius) - w;
        if (x > 0) {
          alpha = std::acos(x / r);
        } else {
          alpha = M_PI_2 + std::acos(-x / r);
        }
        if (!isfinite(alpha)) printf("%f %f %f\n", radius, w, r);
      } else {
        // Point will hit the front of the robot first.
        alpha = std::asin(l / r);
        if (!isfinite(alpha)) printf("%f %f\n", l, r);
      }
      CHECK(isfinite(alpha));
      // if (theta < 0.0f) continue;
      CHECK(std::isfinite(r));
      CHECK(std::isfinite(radius));
      CHECK(std::isfinite(alpha));
      CHECK(std::isfinite(theta));
      const float path_length_ =
          std::max<float>(0.0f, std::fabs(radius) * (theta - alpha));
      if (path->length_ > path_length_) {
        path->length_ = path->length_;
        path->obstruction_ = p;
        angle_min = theta;
        path->obstacle_constrained_ = true;
      }
      if (path->length_ == 0.0) {
        break;
      }
    }
    path->length_ = std::max(0.0f, path->length_);
    if (path->length_ < stopping_dist) {
      path->length_ = 0;
    }
    angle_min =
        std::min<float>(angle_min, path->length_ * std::fabs(path->curvature_));

    for (const Vector2f& p : pointcloud_) {
      const float theta = path->curvature_ > 0.0f
                              ? std::atan2<float>(p.x(), radius - p.y())
                              : std::atan2<float>(p.x(), p.y() - radius);
      if (theta < params_.clearance_clip_fraction_ * angle_min && theta > 0.0) {
        const float r = (p - c).norm();
        const float current_clearance = fabs(r - fabs(radius));
        if (path->clearance_ > current_clearance) {
          path->clearance_ = current_clearance;
        }
      }
    }
    path->clearance_ = std::max(0.0f, path->clearance_);
  }
}

}  // namespace motion
