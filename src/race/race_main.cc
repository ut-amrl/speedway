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

#include "track/track_model.h"

namespace {
DEFINE_string(config, "config/race.lua", "path to config file");

CONFIG_STRING(odom_topic, "RaceParameters.odom_topic");
CONFIG_STRING(laser_topic, "RaceParameters.laser_topic");

std::unique_ptr<track::TrackModel> track_model_;
}  // namespace

using Eigen::Vector2f;

void OdomCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v) {
    LOG(INFO) << "Odometry t=" << msg.header.stamp.toSec();
  }
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v) {
    LOG(INFO) << "Laser t=" << msg.header.stamp.toSec()
              << ",\tdf=" << GetWallTime() - msg.header.stamp.toSec();
  }

  static std::vector<Vector2f> cloud;
  cloud.clear();

  for (size_t i = 0; i < (msg.angle_max - msg.angle_min) / msg.angle_increment;
       i++) {
    if (msg.ranges[i] <= msg.range_min || msg.ranges[i] >= msg.range_max) {
      continue;
    }
    double angle = msg.angle_min + msg.angle_increment * i;
    cloud.push_back(
        Vector2f{msg.ranges[i] * cos(angle) + 0.2, msg.ranges[i] * sin(angle)});
  }

  track_model_->UpdatePointcloud(cloud);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  config_reader::ConfigReader config_reader({FLAGS_config});

  ros::init(argc, argv, "race");
  ros::NodeHandle node_handle;

  ros::Subscriber odom_sub =
      node_handle.subscribe(CONFIG_odom_topic, 1, &OdomCallback);
  ros::Subscriber laser_sub =
      node_handle.subscribe(CONFIG_laser_topic, 1, &LaserCallback);

  track_model_ = std::make_unique<track::TrackModel>();

  ros::Rate loop(40);
  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
