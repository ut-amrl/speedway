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
#include "visualization/visualization.h"

using Eigen::Vector2f;

namespace {
DEFINE_string(config, "config/race.lua", "path to config file");

CONFIG_STRING(odom_topic, "RaceParameters.odom_topic");
CONFIG_STRING(laser_topic, "RaceParameters.laser_topic");

CONFIG_VECTOR2F(laser_location, "RaceParameters.laser_location");

CONFIG_UINT(wall_color, "RaceParameters.wall_color");
CONFIG_UINT(wall_polynomial_order, "RaceParameters.wall_polynomial_order");
CONFIG_DOUBLE(wall_tolerance, "RaceParameters.wall_tolerance");

std::unique_ptr<track::TrackModel> track_model_;

ros::Publisher viz_pub_;
amrl_msgs::VisualizationMsg local_viz_msg_;
amrl_msgs::VisualizationMsg global_viz_msg_;
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

  static std::vector<Vector2f> cloud;
  cloud.clear();

  for (size_t i = 0; i < (msg.angle_max - msg.angle_min) / msg.angle_increment;
       i++) {
    if (msg.ranges[i] <= msg.range_min || msg.ranges[i] >= msg.range_max) {
      continue;
    }
    double angle = msg.angle_min + msg.angle_increment * i;
    cloud.push_back(
        Vector2f{msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)} +
        CONFIG_laser_location);
  }

  track_model_->UpdatePointcloud(cloud, msg.angle_min, msg.angle_max,
                                 msg.angle_increment);
}

void DrawWallCurves() {
  std::vector<Vector2f> left_wall = track_model_->SampleLeftWall();
  std::vector<Vector2f> right_wall = track_model_->SampleRightWall();

  if (left_wall.size() > 1)
    for (size_t i = 1; i < left_wall.size(); i++) {
      visualization::DrawLine(left_wall[i - 1], left_wall[i], CONFIG_wall_color,
                              local_viz_msg_);
    }

  if (right_wall.size() > 1)
    for (size_t i = 1; i < right_wall.size(); i++) {
      visualization::DrawLine(right_wall[i - 1], right_wall[i],
                              CONFIG_wall_color, local_viz_msg_);
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

  viz_pub_ =
      node_handle.advertise<amrl_msgs::VisualizationMsg>("visualization", 1);

  track_model_ = std::make_unique<track::TrackModel>(
      CONFIG_wall_polynomial_order, CONFIG_wall_tolerance);

  local_viz_msg_ =
      visualization::NewVisualizationMessage("base_link", "race_local");
  global_viz_msg_ =
      visualization::NewVisualizationMessage("map", "race_global");

  ros::Rate loop(40);
  while (ros::ok()) {
    ros::spinOnce();

    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    DrawWallCurves();

    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);

    loop.sleep();
  }

  return 0;
}
