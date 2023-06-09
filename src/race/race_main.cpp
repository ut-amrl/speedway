#include <amrl_msgs/AckermannCurvatureDriveMsg.h>
#include <config_reader/config_reader.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <shared/util/timer.h>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "race/race.hpp"
#include "visualization/visualization.h"

namespace {
DEFINE_string(config, "config/race.lua", "path to config file");

CONFIG_STRING(odom_topic, "RaceParameters.ros_topics.odom");
CONFIG_STRING(laser_topic, "RaceParameters.ros_topics.laser");
CONFIG_STRING(vis_topic, "RaceParameters.ros_topics.visualization");
CONFIG_STRING(ackermann_topic, "RaceParameters.ros_topics.ackermann");

CONFIG_VECTOR2F(laser_location, "RaceParameters.laser_config.laser_location");
CONFIG_BOOL(include_out_of_range,
            "RaceParameters.laser_config.include_out_of_range");

ros::Publisher ackermann_pub_;
amrl_msgs::AckermannCurvatureDriveMsg ackermann_msg_;

ros::Publisher viz_pub_;
amrl_msgs::VisualizationMsg local_viz_msg_;
amrl_msgs::VisualizationMsg global_viz_msg_;

Race race_;
}  // namespace

void OdomCallback(const nav_msgs::Odometry& msg) {
  VLOG(2) << "Odometry t=" << msg.header.stamp.toSec();
  race_.UpdateOdometry(
      Eigen::Vector2f{msg.pose.pose.position.x, msg.pose.pose.position.y},
      2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Eigen::Vector2f{msg.twist.twist.linear.x, msg.twist.twist.linear.y},
      msg.twist.twist.angular.z);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  VLOG(2) << "Laser t=" << msg.header.stamp.toSec()
          << ",\tdf=" << GetWallTime() - msg.header.stamp.toSec();

  static std::vector<Eigen::Vector2f> cloud;
  cloud.clear();

  for (size_t i = 0; i < (msg.angle_max - msg.angle_min) / msg.angle_increment;
       i++) {
    if (msg.ranges[i] <= msg.range_min || msg.ranges[i] >= msg.range_max) {
      if (CONFIG_include_out_of_range)
        cloud.push_back(
            Eigen::Vector2f{
                msg.range_max * cos(msg.angle_min + msg.angle_increment * i),
                msg.range_max * sin(msg.angle_min + msg.angle_increment * i)} +
            CONFIG_laser_location);
      else
        continue;
    }
    double angle = msg.angle_min + msg.angle_increment * i;
    cloud.push_back(Eigen::Vector2f{msg.ranges[i] * cos(angle),
                                    msg.ranges[i] * sin(angle)} +
                    CONFIG_laser_location);
  }

  race_.UpdateLaser(cloud);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  config_reader::ConfigReader config_reader({FLAGS_config});
  VLOG(1) << "Loaded config from " << FLAGS_config;

  ros::init(argc, argv, "race");
  ros::NodeHandle node_handle;

  ros::Subscriber odom_sub =
      node_handle.subscribe(CONFIG_odom_topic, 1, &OdomCallback);
  ros::Subscriber laser_sub =
      node_handle.subscribe(CONFIG_laser_topic, 1, &LaserCallback);

  ackermann_pub_ = node_handle.advertise<amrl_msgs::AckermannCurvatureDriveMsg>(
      CONFIG_ackermann_topic, 1);
  ackermann_msg_.header.frame_id = "base_link";
  ackermann_msg_.header.seq = 0;
  ackermann_msg_.header.stamp = ros::Time::now();

  viz_pub_ =
      node_handle.advertise<amrl_msgs::VisualizationMsg>(CONFIG_vis_topic, 1);
  local_viz_msg_ =
      visualization::NewVisualizationMessage("base_link", "race_local");
  global_viz_msg_ =
      visualization::NewVisualizationMessage("map", "race_global");

  LOG(INFO) << "Starting...";
  ros::Rate loop(40);
  while (ros::ok()) {
    ros::spinOnce();

    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    float speed, curvature;
    if (!race_.Run(speed, curvature)) {
      LOG(ERROR) << "Error in Race::Run.";
    } else {
      ackermann_msg_.header.seq++;
      ackermann_msg_.header.stamp = ros::Time::now();
      ackermann_msg_.velocity = speed;
      ackermann_msg_.curvature = curvature;
      ackermann_pub_.publish(ackermann_msg_);
    }

    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);

    loop.sleep();
  }

  return 0;
}
