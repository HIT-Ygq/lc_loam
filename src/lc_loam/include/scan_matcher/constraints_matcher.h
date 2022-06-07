#pragma once

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include "graph_node.h"
#include "pose_graph_data.h"
#include "scan_matcher/scan_to_scan_matcher.h"

namespace optimization{

int findNearestScan(const mapping::SubmapId& submap_id,
                     const mapping::NodeId& node_id,
                     const mapping::PoseGraphData& data,
                     const mapping::TrajectoryNode::Data* const constant_data,
                     const transform::Rigid3d& global_pose,
                     int num_range_data,
                     double& min_distance,
                     int submap_threshold,
                     int node_id_threshold);

transform::Rigid3d icpScanMatch(const mapping::PoseGraphData& data,
                 const mapping::TrajectoryNode::Data* const constant_data,
                 int min_scan_id,
                 bool& hasConverged);

transform::Rigid3d ndtScanMatcher(const mapping::PoseGraphData& data,
                 const mapping::TrajectoryNode::Data* const constant_data,
                 const Submap* submap,
                 int min_scan_id,
                 bool* hasConverged);
}