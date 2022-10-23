#include <config_reader/config_reader.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace {

DEFINE_string(config, "config/race.lua", "path to config file");

}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  config_reader::ConfigReader config_reader({FLAGS_config});

  ros::init(argc, argv, "race");
  ros::NodeHandle node_handle;

  ros::Rate loop(40);
  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
