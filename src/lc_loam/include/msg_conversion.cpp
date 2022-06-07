#include "msg_conversion.h"

namespace transform{

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
    geometry_msgs::Point point;
    point.x = vector3d.x();
    point.y = vector3d.y();
    point.z = vector3d.z();
    return point;
}

void transformPointData(pcl::PointXYZI const *const pi,
                        pcl::PointXYZI *const po,
                        const transform::Rigid3d &transform_){
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = transform_.rotation() * point_curr +
                                                  transform_.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
}

void tranformRageData(
              const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in_origin,
		      pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in_origin_local,
              const transform::Rigid3d& pose_estimate){
    for (int i = 0; i < (int)points_in_origin->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        transform::transformPointData(&(points_in_origin
        ->points[i]), &point_temp, pose_estimate);
        points_in_origin_local->push_back(point_temp);
    }
}

bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}