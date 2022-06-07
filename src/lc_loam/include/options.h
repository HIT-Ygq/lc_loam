#pragma once

#include <string>
#include "config/lua_parameter_dictionary.h"
#include "config/port.h"

namespace common{

    struct LidarOptions{
        double max_distance;
        double min_distance;
        int num_lines = 64;
        double scan_period;
        int points_per_line;
        double horizontal_angle_resolution;
        double horizontal_angle;
        double vertical_angle_resolution;
        double vertical_angle;
    };
    struct OdomOptions{
            double map_resolution_edge = 0.4;
            double map_resolution_surf = 0.6;
            int num_range_data = 40;

            int num_thread_pool = 4;
            double speed_filter = 0.1;
            double scan_period = 0.1;
    };

    struct LoopClosureOptions{
           int num_range_data = 40;
           double loop_closure_translation_weight;
           double loop_closure_rotation_weight;
           int submap_threshold;
           int node_id_threshold;
           double scan_to_scan_min;
           double distance_threshold = 50;
           int num_of_submap_threshold = 2;
    };

    struct PoseGraphOptions{
           double matcher_translation_weight = 500.0;
           double matcher_rotation_weight = 1600.0;
           int optimize_every_n_nodes = 120;
           bool fix_z_in_3d = false;

           LoopClosureOptions loop_closureOptions;
    };

    class NodeOptions{
        public:
            LidarOptions lidar_param_;
            OdomOptions odom_options;
            PoseGraphOptions pose_graph_options;
    };

    NodeOptions CreateNodeOptions(
    common::LuaParameterDictionary* lua_parameter_dictionary);

    NodeOptions LoadOptions(const std::string &configuration_directory,
                        const std::string &configuration_basename);


}