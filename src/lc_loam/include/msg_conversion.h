#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

#include "geometry_msgs/Pose.h"

#include "common/transform.h"

namespace transform{

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d &vector3d);

void transformPointData(pcl::PointXYZI const *const pi,
                        pcl::PointXYZI *const po,
                        const transform::Rigid3d &transform_);

void tranformRageData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in_origin,
		                    pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in_origin_local,
							const transform::Rigid3d& pose_estimate);

bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name);

}