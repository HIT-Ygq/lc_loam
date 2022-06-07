#include "constraints_matcher.h"

namespace optimization{

int findNearestScan(const mapping::SubmapId& submap_id,
                     const mapping::NodeId& node_id,
                     const mapping::PoseGraphData& data,
                     const mapping::TrajectoryNode::Data* const constant_data,
                     const transform::Rigid3d& global_pose,
                     int num_range_data,
                     double& min_distance,
                     int submap_threshold,
                     int node_id_threshold){
        int min_scan_id = 0;
        transform::Rigid3d first_scan;
        min_scan_id = num_range_data * submap_id.submap_index + num_range_data;

        if(submap_id.submap_index <= submap_threshold && node_id.node_index < node_id_threshold){
            for (int i = num_range_data * submap_id.submap_index;
            i != num_range_data * submap_id.submap_index + 2 * num_range_data; ++i)
           {
            first_scan =
            data.trajectory_nodes.at(mapping::NodeId{0, i}).constant_data->local_pose;
            if((constant_data->local_pose.translation() -
                        first_scan.translation()).norm() < min_distance){
                 min_distance = (constant_data->local_pose.translation() -
                                     first_scan.translation())
                                        .norm();
                 min_scan_id = i;
            }
            }
        }
        else{
        for (int i = num_range_data * submap_id.submap_index;
            i != num_range_data * submap_id.submap_index + 2 * num_range_data; ++i)
        {
            first_scan =
            data.trajectory_nodes.at(mapping::NodeId{0, i}).global_pose;
            if((global_pose.translation() -
                        first_scan.translation()).norm() < min_distance){
                 min_distance = (global_pose.translation() -
                                     first_scan.translation())
                                        .norm();
                 min_scan_id = i;
            }
            }
        }

       return min_scan_id;
}

transform::Rigid3d icpScanMatch(const mapping::PoseGraphData& data,
                 const mapping::TrajectoryNode::Data* const constant_data,
                 int min_scan_id,
                 bool& hasConverged){
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        *input_cloud = *constant_data->range_data_raw.edge_data_in_local +
                       *constant_data->range_data_raw.surf_data_in_local;
        target_cloud->reserve(data.trajectory_nodes.at(
                                   mapping::NodeId{0, min_scan_id})
                                  .constant_data->range_data_raw
                                  .edge_data_in_local->points.size() +
                               data.trajectory_nodes.at(
                                   mapping::NodeId{0, min_scan_id})
                                  .constant_data->range_data_raw
                                  .surf_data_in_local->points.size());
        *target_cloud = *(data.trajectory_nodes.at(
                                   mapping::NodeId{0, min_scan_id})
                                  .constant_data->range_data_raw.edge_data_in_local) +
                                  *(data.trajectory_nodes.at(
                                  mapping::NodeId{0, min_scan_id})
                                  .constant_data->range_data_raw.surf_data_in_local);
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(
            new pcl::PointCloud<pcl::PointXYZI>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false)
            hasConverged = false;

        Eigen::Matrix4d icp_constraint = icp.getFinalTransformation().cast<double>();
        Eigen::Matrix3d rotation_mat = icp_constraint.block(0, 0, 3, 3);
        Eigen::Quaterniond rotation_estimate(rotation_mat);
        Eigen::Vector3d translation_estimate(icp_constraint.block(0, 3, 3, 1));
        transform::Rigid3d transform_scan_to_scan(translation_estimate, rotation_estimate);
        return transform_scan_to_scan;
}

transform::Rigid3d ndtScanMatcher(const mapping::PoseGraphData& data,
                 const mapping::TrajectoryNode::Data* const constant_data,
                 const Submap* submap,
                 int min_scan_id,
                 bool* hasConverged){
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(
             new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());

        *input_cloud = *constant_data->range_data_raw.edge_data_in_local +
                       *constant_data->range_data_raw.surf_data_in_local;
        *target_cloud = *(submap->conerMap()) + *(submap->surfMap());

        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setStepSize(0.1);
        ndt.setResolution(1);
        ndt.setMaximumIterations(50);

        ndt.setInputSource(input_cloud);
        ndt.setInputTarget(target_cloud);
        transform::Rigid3d initial_constraint_ = data.trajectory_nodes.at(
                                                 mapping::NodeId{0, min_scan_id})
                                                .constant_data->local_pose;
        Eigen::Matrix3f inital_rotation_ =
            initial_constraint_.rotation().toRotationMatrix().cast<float>();
        Eigen::Matrix4f initial_constraint_mat = Eigen::Matrix4f::Identity();
        initial_constraint_mat.block(0, 0, 3, 3) = inital_rotation_;
        initial_constraint_mat.block(0, 3, 3, 1) =
            initial_constraint_.translation().cast<float>();
         pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        ndt.align(*output_cloud, initial_constraint_mat);
        if (!ndt.hasConverged())
            *hasConverged = false;

        Eigen::Matrix4d initial_transform = ndt.getFinalTransformation().cast<double>();

        Eigen::Matrix3d rotation_mat = initial_transform.block(0, 0, 3, 3);
        Eigen::Quaterniond rotation_estimate(rotation_mat);
        Eigen::Vector3d translation_estimate(initial_transform.block(0, 3, 3, 1));
        transform::Rigid3d transform_estimate(translation_estimate, rotation_estimate);
        return transform_estimate;
}
}