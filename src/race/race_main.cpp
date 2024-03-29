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

#include "visualization/visualization.h"

using Eigen::Vector2f;

namespace {
DEFINE_string(config, "config/race.lua", "path to config file");

CONFIG_STRING(odom_topic, "RaceParameters.ros_topics.odom");
CONFIG_STRING(laser_topic, "RaceParameters.ros_topics.laser");
CONFIG_STRING(vis_topic, "RaceParameters.ros_topics.visualization");

CONFIG_VECTOR2F(laser_location, "RaceParameters.laser_config.laser_location");
CONFIG_BOOL(include_out_of_range,
            "RaceParameters.laser_config.include_out_of_range");

ros::Publisher viz_pub_;
amrl_msgs::VisualizationMsg local_viz_msg_;
amrl_msgs::VisualizationMsg global_viz_msg_;
}  // namespace

void OdomCallback(const nav_msgs::Odometry& msg) {
  VLOG(2) << "Odometry t=" << msg.header.stamp.toSec();
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  VLOG(2) << "Laser t=" << msg.header.stamp.toSec()
          << ",\tdf=" << GetWallTime() - msg.header.stamp.toSec();

  static std::vector<Vector2f> cloud;
  cloud.clear();

  for (size_t i = 0; i < (msg.angle_max - msg.angle_min) / msg.angle_increment;
       i++) {
    if (msg.ranges[i] <= msg.range_min || msg.ranges[i] >= msg.range_max) {
      if (CONFIG_include_out_of_range)
        cloud.push_back(
            Vector2f{
                msg.range_max * cos(msg.angle_min + msg.angle_increment * i),
                msg.range_max * sin(msg.angle_min + msg.angle_increment * i)} +
            CONFIG_laser_location);
      else
        continue;
    }
    double angle = msg.angle_min + msg.angle_increment * i;
    cloud.push_back(
        Vector2f{msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)} +
        CONFIG_laser_location);
  }
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

    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);

    loop.sleep();
  }

  return 0;
}
