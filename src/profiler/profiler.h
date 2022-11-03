#include <amrl_msgs/AckermannCurvatureDriveMsg.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#ifndef PROFILER_H
#define PROFILER_H

using namespace std;
using namespace Eigen;
using amrl_msgs::AckermannCurvatureDriveMsg;

namespace profiler {

struct CommandHistory {
    Vector2f robot_vel;
    amrl_msgs::AckermannCurvatureDriveMsg msg;
    float angle;
};

struct History {
    Vector2f prev_lidar_loc_; // previous LIDAR location for where sensors were read
    float prev_lidar_angle; // previous robot angle
    vector<CommandHistory> prev_commands; // previously sent drive messages
};

struct PathOption {
    float curvature;
    float clearance;
    float free_path_length;
    Vector2f obstruction;
    Vector2f closest_point;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Profiler {
public:
    Profiler(string profiler_config);
    double DetectObstacles(double curvature, double forward_predicted_dist);
    double ForwardPredict(float actuation_latency);
    void UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud,
        const float angle_min, const float angle_max,
        const float increment);
    void UpdateOdometry(const Vector2f& loc,
        float angle,
        const Vector2f& vel,
        float ang_vel);
    double toc(double distance_left, double time_delta);
    void Profile();

    const double CLEARANCE = 0.1;
    const double BASETOFRONT = 0.4;
    const double HALFWIDTH = 0.1405;

protected:
    std::vector<Vector2f> cloud_;

    // current velocity
    Eigen::Vector2f robot_vel_;
    // current pos
    Eigen::Vector2f odom_loc_;
    // current heading
    float odom_angle_;
    // odom initialized location
    Eigen::Vector2f odom_start_loc_;
    // odom initialized angle
    float odom_start_angle_;
    // whether odometry has been initialized
    bool odom_initialized_;
    // current angular speed
    float robot_omega_;

    History history;

    ros::Publisher drive_pub_;
};

} // namespace profiler

#endif // PROFILER_H
