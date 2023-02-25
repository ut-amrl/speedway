#include <eigen3/Eigen/Dense>

namespace race {

class Race {
 private:
  bool odom_initialized_ = false;

 public:
  Eigen::Vector2f position_;
  float angle_;
  Eigen::Vector2f velocity_;

  void UpdateOdometry(const Eigen::Vector3f& odom_loc,
                      const Eigen::Quaternionf& odom_quat);

  void Run(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
};

}  // namespace race