#include <config_reader/config_reader.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <shared/util/timer.h>

namespace {

DEFINE_string(config, "config/race.lua", "path to config file");

CONFIG_STRING(odom_topic, "RaceParameters.odom_topic");
CONFIG_STRING(laser_topic, "RaceParameters.laser_topic");

}  // namespace

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

  ros::Rate loop(40);
  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
