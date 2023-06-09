#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

namespace motion {
// State of the robot relevant to the local planner
struct LocalPlannerState {
  // Location of robot according to odometry (in odom frame)
  Eigen::Vector2f odom_loc;
  // Angle of robot according to odometry (in odom frame)
  float odom_angle;

  // Linear velocity of robot according to odometry (in odom frame)
  Eigen::Vector2f odom_vel;
  // Angular velocity of robot according to odometry (in odom frame)
  float odom_ang_vel;

  // Raw pointcloud in base_link frame
  std::vector<Eigen::Vector2f> pointcloud;
};

// A trajectory in the robot's local frame
class TrajectoryBase {
 public:
  // Arc length of the trajectory
  virtual float Length() const = 0;

  // Endpoint of the trajectory
  virtual Eigen::Vector2f EndPoint() const = 0;

  // Total angular difference between the start and end of the trajectory
  virtual float AngularLength() const = 0;

  // Get the interpolated point where t \in [0, 1] represents the fraction of
  // the trajectory
  virtual Eigen::Vector2f AtT(float t) const = 0;
  // Get the interpolated point where s is the traversed arc length. If s is
  // greater than the length of the trajectory, the endpoint of the trajectory
  // will be returned.
  virtual Eigen::Vector2f AtDistance(float s) const = 0;
};

// A sampler generates a set of trajectories from a given state
// TODO: see if we need to pass the trajectories as pointers
class SamplerBase {
 public:
  // Returns a set of trajectories from the given state
  virtual std::vector<std::shared_ptr<TrajectoryBase>> Sample(
      const LocalPlannerState& state) const = 0;
};

// An evaluator scores a trajectory based on a given state
class EvaluatorBase {
 public:
  // Returns a score for the given trajectory and state
  virtual float Evaluate(const std::shared_ptr<TrajectoryBase>& trajectory,
                         const LocalPlannerState& state) = 0;
  // Returns the best trajectory from the given set of trajectories and state
  virtual std::shared_ptr<TrajectoryBase> FindBest(
      const std::vector<std::shared_ptr<TrajectoryBase>>& trajectories,
      const LocalPlannerState& state) const = 0;
};
}  // namespace motion
