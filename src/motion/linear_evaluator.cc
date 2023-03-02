#include "motion/linear_evaluator.h"

#include <float.h>

#include <vector>

#include "math/line2d.h"
#include "motion/constant_curvature_arc.h"

namespace motion {

LinearEvaluator::LinearEvaluator(
    const MotionParameters& motion_params,
    const LinearEvaluatorParameters& evaluator_params)
    : PathEvaluatorBase(motion_params), evaluator_params_(evaluator_params) {}

std::shared_ptr<PathOptionBase> LinearEvaluator::FindBest(
    const std::vector<std::shared_ptr<PathOptionBase>>& paths) {
  if (paths.size() == 0) return nullptr;

  // Check if there is any path with an obstacle-free path from the end to the
  // local target.
  std::vector<float> clearance_to_goal(paths.size(), 0.0);
  std::vector<float> dist_to_goal(paths.size(), FLT_MAX);
  bool path_to_goal_exists = false;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto endpoint = paths[i]->EndPoint().translation;
    clearance_to_goal[i] =
        StraightLineClearance(geometry::Line2f(endpoint, goal_), pointcloud_);
    if (clearance_to_goal[i] > 0.0) {
      dist_to_goal[i] = (endpoint - goal_).norm();
      path_to_goal_exists = true;
    }
  }
  // If there is no path with an obstacle-free path from the end to the local
  // target, then reset all distance to goals.
  if (!path_to_goal_exists) {
    for (size_t i = 0; i < paths.size(); ++i) {
      const auto endpoint = paths[i]->EndPoint().translation;
      dist_to_goal[i] = (endpoint - goal_).norm();
    }
  }

  // First find the shortest path.
  std::shared_ptr<PathOptionBase> best = nullptr;
  float best_path_length = FLT_MAX;
  int best_i = 0;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length = paths[i]->Length() + dist_to_goal[i];
    if (path_length < best_path_length) {
      best_path_length = path_length;
      best = paths[i];
      best_i = i;
    }
  }

  // Recompute clearance as the local weighted mean of the fpl around and
  // including the current option.
  std::vector<float> clearance(paths.size());
  std::vector<float> option_clearance(paths.size());
  const int n =
      ceil(static_cast<float>(paths.size()) *
           evaluator_params_.fpl_avg_windows_[evaluator_params_.active_index_]);
  const float w = 1.0f / static_cast<float>(n);
  for (size_t i = 0; i < paths.size(); ++i) {
    clearance[i] = w * paths[i]->Length();
    option_clearance[i] = paths[i]->Clearance();
    for (int j = 1; j < n && i + j < paths.size() && static_cast<int>(i) >= j;
         ++j) {
      if (paths[i - j]->Length() <= 0.0f || paths[i + j]->Length() <= 0.0f)
        break;
      clearance[i] += w * paths[i - j]->Length();
      clearance[i] += w * paths[i + j]->Length();
    }
    if (FLAGS_v > 2) {
      printf(
          "%3d: curvature=%7.3f, fpl=%7.3f, avg_fpl=%7.3f, "
          "option_clearance=%4.1f clearance=%7.3f, dist_to_goal=%7.3f\n",
          static_cast<int>(i),
          reinterpret_cast<const ConstantCurvatureArc*>(paths[i].get())
              ->curvature_,
          paths[i]->Length(), clearance[i], option_clearance[i],
          paths[i]->Clearance(),
          dist_to_goal[i] < FLT_MAX ? dist_to_goal[i] : 1.0 / 0.0f);
    }
  }

  if (best == nullptr) {
    // No valid paths!
    return nullptr;
  }

  // Next try to find better paths.
  float best_cost = CalculateCost(dist_to_goal[best_i], best->Length(),
                                  option_clearance[best_i], clearance[best_i]);
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    // const float path_length = paths[i]->Length() + dist_to_goal[i];
    // const float cost = FLAGS_dw * path_length +
    //   FLAGS_fw * paths[i]->Length() +
    //   FLAGS_cw * paths[i]->Clearance();
    const float cost = CalculateCost(dist_to_goal[i], paths[i]->Length(),
                                     option_clearance[i], clearance[i]);
    if (cost < best_cost) {
      best = paths[i];
      best_cost = cost;
    }
  }
  return best;
}

float LinearEvaluator::CalculateCost(const float dist_to_goal,
                                     const float length,
                                     const float option_clearance,
                                     const float clearance) {
  return evaluator_params_.distance_weights_[evaluator_params_.active_index_] *
             dist_to_goal +
         evaluator_params_.free_path_weights_[evaluator_params_.active_index_] *
             length +
         evaluator_params_
                 .option_clearance_weights_[evaluator_params_.active_index_] *
             option_clearance +
         evaluator_params_.clearance_weights_[evaluator_params_.active_index_] *
             clearance;
}

}  // namespace motion
