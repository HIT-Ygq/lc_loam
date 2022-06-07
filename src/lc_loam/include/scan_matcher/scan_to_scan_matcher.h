#pragma once

#include "common/transform.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ceres_pose.h"
#include "ceres/rotation.h"
#include "ceres_cost_function.h"

#include "ceres_cost_scan_function.h"

namespace optimization{
class CeresScanMatcherScan{

public:
      CeresScanMatcherScan();
      ~CeresScanMatcherScan();

      void match(transform::Rigid3d* initial_pose_estimation,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_cur,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_cur,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_last,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_points_last,
                 ceres::Solver::Summary *const summary);

      void addEdgeFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_cur,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_last,
                         ceres::Problem &problem, ceres::LossFunction *loss_function,
                         double* para_q,
                         double* para_t,
                         int* corner_correspondence);

      void addSurfFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_cur,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_last,
                         ceres::Problem &problem, ceres::LossFunction *loss_function,
                         double* para_q,
                         double* para_t,
                         int* plane_correspondence);

private:
     double DISTANCE_SQ_THRESHOLD = 25;
     double NEARBY_SCAN = 2.5;
};

}