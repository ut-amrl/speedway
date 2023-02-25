#include "race/race.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

namespace race {

void Race::UpdateOdometry(const Eigen::Vector3f& odom_loc,
                          const Eigen::Quaternionf& odom_quat) {
  odom_initialized_ = true;

  position_ = odom_loc.head<2>();
  angle_ = atan2(odom_quat.z(), odom_quat.w());
}

void Race::Run(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel) {
  if (!odom_initialized_) {
    if (FLAGS_v) LOG(INFO) << "Odometry not initialized";
    return;
  }

  cmd_vel = {0, 0};
  cmd_angle_vel = 0;
}

}  // namespace race
