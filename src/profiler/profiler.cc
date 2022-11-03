#include <amrl_msgs/AckermannCurvatureDriveMsg.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <config_reader/config_reader.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include "profiler.h"
#include "visualization/visualization.h"

using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;

namespace profiler {

CONFIG_DOUBLE(max_velocity, "ProfilerParameters.max_velocity");
CONFIG_DOUBLE(max_acceleration, "ProfilerParameters.max_acceleration");
CONFIG_DOUBLE(max_deceleration, "ProfilerParameters.max_deceleration");

float lidarMaxRange = 10;

VisualizationMsg local_viz_msg_;

Profiler::Profiler(string profiler_config)
    : robot_vel_(0, 0)
    , odom_loc_(0, 0)
    , odom_angle_(0)
    , odom_start_loc_(0, 0)
    , odom_start_angle_(0)
    , odom_initialized_(false)
    , robot_omega_(0)
{
    config_reader::ConfigReader config_reader({ profiler_config });

    ros::NodeHandle node_handle;
    drive_pub_ = node_handle.advertise<amrl_msgs::AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
}

double Profiler::ForwardPredict(float actuation_latency)
{
    double forward_predicted_dist = 0;
    Vector2f forward_predicted_disp { 0, 0 }; // TODO: probably need to switch to those in the end
    Vector2f vel_vec;
    Vector2f prev_vel_vec = robot_vel_;
    for (size_t i = 0; i < history.prev_commands.size(); i++) {
        float timestamp_diff;
        if (i + 1 < history.prev_commands.size()) {
            timestamp_diff = (history.prev_commands[i + 1].msg.header.stamp - history.prev_commands[i].msg.header.stamp).toSec();
            timestamp_diff -= actuation_latency;
        } else {
            timestamp_diff = (ros::Time::now() - history.prev_commands[i].msg.header.stamp).toSec();
        }
        vel_vec = history.prev_commands[i].msg.velocity * Vector2f { cos(history.prev_commands[i].angle), sin(history.prev_commands[i].angle) };
        forward_predicted_disp += actuation_latency * prev_vel_vec + timestamp_diff * vel_vec;
        forward_predicted_dist += actuation_latency * prev_vel_vec.norm() + timestamp_diff * history.prev_commands[i].msg.velocity;
        prev_vel_vec = vel_vec;
    }
    return forward_predicted_dist;
}

double Profiler::DetectObstacles(double curvature, double forward_predicted_dist)
{
    double distanceLeft = profiler::lidarMaxRange;

    for (Vector2f point : cloud_) {
        // Straight Line Case
        point[0] -= forward_predicted_dist;
        if (abs(curvature) <= 1e-6) {
            if (abs(point[1]) <= CLEARANCE + HALFWIDTH) {
                double xStoppingDistance = BASETOFRONT + CLEARANCE;
                distanceLeft = std::min(distanceLeft, point[0] - xStoppingDistance);
                visualization::DrawPoint(point, 0x00FF00, local_viz_msg_);
            }
        }
        // Curvies
        // else {
        //   bool clockwise = curvature < 0;
        //   double r = abs(1.0/curvature);

        //   double h = BASETOFRONT + CLEARANCE;
        //   double w = HALFWIDTH + CLEARANCE;
        //   double r_1 = r - w;
        //   double r_2 = sqrt(pow(h, 2) + pow(r + w, 2));

        //   Vector2f c{0, r};
        //   // visualization::DrawCross(Vector2f {0, clockwise ? -r : r}, 0.2, 0xFF00FF, local_viz_msg_);
        //   //visualization::DrawArc(Vector2f {0, clockwise ? -r : r}, r_1, 0, 2 * 3.14, 0x0000000, local_viz_msg_);
        //   //visualization::DrawArc(Vector2f {0, clockwise ? -r : r}, r_2, 0, 2 * 3.14, 0x0000000, local_viz_msg_);
        //   Vector2f drawpoint = point;
        //   if (clockwise) point[1] *= -1;
        //   if(point[0] > 0 && (point - c).norm() >= r_1 && (point - c).norm() <= r_2){
        //     visualization::DrawPoint(drawpoint, 0x00FF00, local_viz_msg_);
        //     double theta = atan2(point[0], r - point[1]);
        //     double omega = atan2(h, r_1);
        //     double phi = theta - omega;
        //     double f = r * phi;
        //     distanceLeft = std::min(distanceLeft, f);
        //     // printf("%f %f\n", f, distanceLeft);
        //   }
        // }
    }
    return distanceLeft;
}

void Profiler::UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud,
    const float angle_min, const float angle_max,
    const float increment)
{
    cloud_ = cloud;
}

double Profiler::toc(double distance_left, double time_delta)
{
    // Calculate acceleration for 1d TOC
    // takes 1/6 m to slow down from 1 m/s at 3 m/s/s
    // need to start decelerating 1/6 m before destination

    float velocity = history.prev_commands.empty() ? robot_vel_.norm() : history.prev_commands.end()->msg.velocity;

    double distanceToStop = -(velocity * velocity) / 2 / CONFIG_max_deceleration;
    double distanceToMaxVelocity = (CONFIG_max_velocity * CONFIG_max_velocity - std::min((velocity * velocity), (float)(CONFIG_max_velocity * CONFIG_max_velocity))) / 2 / CONFIG_max_acceleration;

    // return velocity
    if (distance_left <= distanceToStop) {
        return std::min(CONFIG_max_velocity, std::max(0.0, velocity + time_delta * CONFIG_max_deceleration));
    } else if (distance_left > distanceToMaxVelocity) {
        return std::min(CONFIG_max_velocity, velocity + time_delta * CONFIG_max_acceleration);
    } else {
        return std::min(CONFIG_max_velocity, velocity + time_delta * CONFIG_max_acceleration);
    }
}

void Profiler::UpdateOdometry(const Vector2f& loc,
    float angle,
    const Vector2f& vel,
    float ang_vel)
{
    robot_omega_ = ang_vel;
    robot_vel_ = vel;
    if (!odom_initialized_) {
        odom_start_angle_ = angle;
        odom_start_loc_ = loc;
        odom_initialized_ = true;
        odom_loc_ = loc;
        odom_angle_ = angle;
        return;
    }
    odom_loc_ = loc;
    odom_angle_ = angle;
}

void Profiler::Profile()
{
    static ros::Time last_time;

    auto drive_msg = amrl_msgs::AckermannCurvatureDriveMsg();

    const double actuation_latency = 0.12;
    double forward_predicted_dist = ForwardPredict(actuation_latency);
    double free_path_length = DetectObstacles(0, forward_predicted_dist);

    auto time_delta = (ros::Time::now() - last_time).toSec();
    last_time = ros::Time::now();

    drive_msg.header.stamp = ros::Time::now();
    drive_msg.curvature = 0;
    drive_msg.velocity = toc(free_path_length, time_delta);

    drive_pub_.publish(drive_msg);
}

} // namespace profiler
