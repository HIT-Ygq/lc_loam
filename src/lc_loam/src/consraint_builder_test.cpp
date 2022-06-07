#include <string>
#include <math.h>
#include <vector>
#include <mutex>
#include <string>
#include <fstream>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "pose_graph.h"
#include "graph_node.h"
#include "options.h"
#include "submap/activeSubmap.h"

#include "scan_matcher/scan_matcher.h"
#include "scan_matcher/ceres_cost_function.h"
#include "glog/logging.h"

int main(int argc, char **argv){

    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO);
    optimization::CeresScanMatcher ceres_scan_matcher;
    common::ThreadPool thred_pool(4);
    optimization::ConstraintBuilder constraint_builder(&thred_pool);

    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(
                               new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud(
                               new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_map(
                               new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_map(
                               new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(
        "/home/xcy/pcd/pointcloud/Cornerpoint_135.pcd", *corner_cloud) == -1)//*打开点云文件。
	{
		PCL_ERROR("Couldn't read that pcd file\n");
		return(-1);//如果没找到该文件，返回-1，跳出整个main函数
	}
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(
          "/home/xcy/pcd/pointcloud/Surfpoint_135.pcd", *surf_cloud) == -1)//*打开点云文件。
	{
		PCL_ERROR("Couldn't read that pcd file\n");
		return(-1);//如果没找到该文件，返回-1，跳出整个main函数
	}
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(
          "/home/xcy/pcd/submap/CornerSubmap_16.pcd", *corner_map) == -1)//*打开点云文件。
	{
		PCL_ERROR("Couldn't read that pcd file\n");
		return(-1);//如果没找到该文件，返回-1，跳出整个main函数
	}
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(
        "/home/xcy/pcd/submap/SurfSubmap_16.pcd", *surf_map) == -1)//*打开点云文件。
	{
		PCL_ERROR("Couldn't read that pcd file\n");
		return(-1);//如果没找到该文件，返回-1，跳出整个main函数
	}

    Eigen::Vector3d translation(158.724, -164.879, 6.70596);
    Eigen::Quaterniond rotation(0.741005, -0.0359629, -0.00935738, 0.670471);
    transform::Rigid3d local_submap_pose(translation, rotation);

    Eigen::Vector3d translation_(159.219, -154.368, 3.95429);
    Eigen::Quaterniond rotation_(0.740813, -0.0381523, -0.0139444, 0.670482);
    transform::Rigid3d local_pose(translation_, rotation_);

    Submap submap(local_submap_pose);
    submap.InsertRangeData(corner_map, surf_map);

    mapping::TrajectoryNode::Data constant_data{local_pose, {corner_cloud, surf_cloud}};

    std::deque<std::unique_ptr<mapping::PoseGraphInterface::Constraint>> constraints_;
    constraints_.emplace_back();
    auto* const constraint = &constraints_.back();

    constraint_builder.ComputeConstraint(mapping::SubmapId{0, 13}, mapping::NodeId{0, 155},
                                         false, &constant_data, local_pose,
                                         local_submap_pose, &submap, constraint);

    transform::Rigid3d z_ba = local_submap_pose.inverse() * local_pose;

    std::cout << "z_ba: " << std::endl << z_ba << std::endl;

    //printf("%f\n", (*constraint)->pose.zbar_ij.translation().x());
    std::cout << "constraint: " << std::endl
              << (*constraint)->pose.zbar_ij << std::endl;
}