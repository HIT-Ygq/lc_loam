#pragma once

#include "ceres_pose.h"
#include "ceres_cost_function.h"

#include "submap/submap.h"

namespace optimization
{
    class CeresScanMatcher3D
    {
    public:
        CeresScanMatcher3D();
        ~CeresScanMatcher3D() = default;

        void Match(transform::Rigid3d* initial_pose_estimation,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_edge_in,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_surf_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_in_local,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud_in_local,
                   ceres::Solver::Summary *const summary,
                   bool& matcher_ok);
        void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                               const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                               ceres::Problem &problem, ceres::LossFunction *loss_function,
                               bool &matcher_ok,
                               double* ceres_parameters,
                               Eigen::Map<Eigen::Vector3d>& t_w_curr,
                               Eigen::Map<Eigen::Quaterniond>& q_w_curr );

        void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                               const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                               ceres::Problem &problem, ceres::LossFunction *loss_function,
                               bool& matcher_ok,
                               double* ceres_parameters,
                               Eigen::Map<Eigen::Vector3d>& t_w_curr,
                               Eigen::Map<Eigen::Quaterniond>& q_w_curr );
    private:
        void transformRangeData(pcl::PointXYZI const *const pi,
                                pcl::PointXYZI *const po,
                                Eigen::Map<Eigen::Vector3d>& t_w_curr,
                                Eigen::Map<Eigen::Quaterniond>& q_w_curr );
        double huberLoss_option = 0.1;
    };
}