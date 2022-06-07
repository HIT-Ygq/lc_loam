#pragma once

#include "ceres_pose.h"
#include "ceres_cost_function.h"

#include "submap/submap.h"

namespace optimization
{
    class CeresScanMatcher
    {
    public:
        CeresScanMatcher();
        ~CeresScanMatcher() = default;

        void setKdtree(const Submap *submap);
        void Match(transform::Rigid3d* initial_pose_estimation,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_edge_in,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_surf_in,
                   const Submap *submap,
                   ceres::Solver::Summary *const summary,
                   bool& matcher_ok);
        void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                               const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                               ceres::Problem &problem, ceres::LossFunction *loss_function,
                               bool& matcher_ok);

        void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                               const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                               ceres::Problem &problem, ceres::LossFunction *loss_function,
                               bool& matcher_ok);

        void transformRangeData(pcl::PointXYZI const *const pi,
                                                 pcl::PointXYZI *const po);

    private:
        double huberLoss_option = 0.1;
        double ceres_parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        Eigen::Map<Eigen::Quaterniond> q_w_curr =
                                    Eigen::Map<Eigen::Quaterniond>(ceres_parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr =
                                    Eigen::Map<Eigen::Vector3d>(ceres_parameters + 4);
        // kd-tree
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap;
    };
}