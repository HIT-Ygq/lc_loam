#include "submap/submap.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr Submap::conerMap() const
{
    return edgeSubmap_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Submap::surfMap() const
{
    return surfSubmap_;
}


void Submap::InsertRangeData(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_in_local,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_in_local){
        *edgeSubmap_ += *edge_points_in_local;
        *surfSubmap_ += *surf_points_in_local;

        downSizeFilterEdge.setInputCloud(edgeSubmap_);
        downSizeFilterEdge.filter(*edgeSubmap_);
        downSizeFilterSurf.setInputCloud(surfSubmap_);
        downSizeFilterSurf.filter(*surfSubmap_);

        set_num_range_data(num_range_data() + 1);
}