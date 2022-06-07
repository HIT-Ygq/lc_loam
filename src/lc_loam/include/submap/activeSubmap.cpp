#include "submap/activeSubmap.h"

ActiveSubmaps::ActiveSubmaps(){
}

std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::submaps() const
{
   return std::vector<std::shared_ptr<const Submap>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::InsertRangeData(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_in_local,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_in_local,
      const transform::Rigid3d& pose_esitimate)
      {

      if(submaps_.empty()){
          AddSubmap(pose_esitimate);
      }
      else if (submaps_.back()->num_range_data() == num_range_data) {
          submaps_.back()->set_center_pose(pose_esitimate);
          AddSubmap(pose_esitimate);
       }
       for(auto& submap : submaps_){
           submap->InsertRangeData(edge_points_in_local,
                                  surf_points_in_local);
       }

       if(submaps_.front()->num_range_data() == 2 * num_range_data)
       {
           submaps_.front()->Finish();
       }
       return submaps();
      }

void ActiveSubmaps::AddSubmap(const transform::Rigid3d &origin)
{
    if(submaps_.size() >= 2)
    {
        CHECK(submaps_.front()->insertion_finished());
        submaps_.erase(submaps_.begin());
    }

    submaps_.push_back(std::make_shared<Submap>(origin,
                              map_resolution_edge, map_resolution_surf));
}

void ActiveSubmaps::set_num_range_data(int num_range_data_){
    num_range_data = num_range_data_;
}

void ActiveSubmaps::set_edge_resolution(double map_resolution_edge_){
    map_resolution_edge = map_resolution_edge_;
}
void ActiveSubmaps::set_surf_resolution(double map_resolution_surf_){
    map_resolution_surf = map_resolution_surf_;
}