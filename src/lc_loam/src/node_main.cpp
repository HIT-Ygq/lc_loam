
//c++ lib
#include <cmath>
#include <vector>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>

//local lib
#include "options.h"
#include "trajectory_builder.h"
#include "glog/logging.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO);
    google::ParseCommandLineFlags(&argc, &argv, true);
    LOG(INFO) << "start!";

    ros::init(argc, argv, "main_1");

    ::common::NodeOptions node_options =
        ::common::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    LocalTrajectoryBuilder trajectory_builder(node_options);

    ros::Rate(8);
    while(ros::ok()){
        trajectory_builder.odom_estimation();
        if(trajectory_builder.optimization_flag)
            {
                trajectory_builder.pose_graph_->lastOptimization();
                LOG(INFO) << "begin to build map!";
                trajectory_builder.optimization_flag = false;
                trajectory_builder.lastMap();
            }
        ros::spinOnce();
    }

    return 0;
}

