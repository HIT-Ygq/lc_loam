#pragma once

#include "submap/submap.h"
#include "glog/logging.h"

class ActiveSubmaps
{
public:
  ActiveSubmaps();
  ActiveSubmaps(const ActiveSubmaps&) = delete;
  ActiveSubmaps& operator=(const ActiveSubmaps&) = delete;

  std::vector<std::shared_ptr<const Submap>> InsertRangeData(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_in_local,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_in_local,
      const transform::Rigid3d& pose_esitimate);

  std::vector<std::shared_ptr<const Submap>> submaps() const;

  void set_num_range_data(int num_range_data_);
  void set_edge_resolution(double map_resolution_edge);
  void set_surf_resolution(double map_resolution_edge);

private:
  int num_range_data = 40;
  double map_resolution_edge = 0.4;
  double map_resolution_surf = 0.8;
  void FinishSubmap();
  void AddSubmap(const transform::Rigid3d &origin);

  std::vector<std::shared_ptr<Submap>> submaps_;

};