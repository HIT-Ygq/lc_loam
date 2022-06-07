#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


#include "common/transform.h"

class Submap
{
public:
    Submap(const transform::Rigid3d& local_submap_pose,
          double map_resolution_edge,
          double map_resolution_surf)
    : map_resolution_edge_(map_resolution_edge),
      map_resolution_surf_(map_resolution_surf),
      local_pose_(local_submap_pose),
     edgeSubmap_(new pcl::PointCloud<pcl::PointXYZI>()),
     surfSubmap_(new pcl::PointCloud<pcl::PointXYZI>())
     {
         downSizeFilterEdge.setLeafSize(map_resolution_edge,
                                    map_resolution_edge, map_resolution_edge);
         downSizeFilterSurf.setLeafSize(
             map_resolution_surf, map_resolution_surf, map_resolution_surf);
     }

    //返回local坐标系子图坐标
    transform::Rigid3d local_pose() const { return local_pose_; }
    transform::Rigid3d center_pose() const { return center_pose_; }


    // 插入点云子图中的雷达数据数
    int num_range_data() const { return num_range_data_; }
    void set_num_range_data(const int num_range_data)
    {
        num_range_data_ = num_range_data;
    }
    void set_center_pose(const transform::Rigid3d& center){
        center_pose_ = center;
    }

    // 返回完成状态
    bool insertion_finished() const { return insertion_finished_;  }

    void set_insertion_finished(bool insertion_finished)
    {
        insertion_finished_ = insertion_finished;
    }

    // 将雷达数据写入到地图，传入原点位于匹配位姿处的点云
    void InsertRangeData(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_in_local,
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_in_local);
    pcl::PointCloud<pcl::PointXYZI>::Ptr conerMap() const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfMap() const;
    // 结束子图

    void Finish() { set_insertion_finished(true); }

private:
    double map_resolution_edge_ = 0.4;
    double map_resolution_surf_ = 0.8;
    const transform::Rigid3d local_pose_;
    transform::Rigid3d center_pose_;
    int num_range_data_ = 0;
    bool insertion_finished_ = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeSubmap_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfSubmap_;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

};
