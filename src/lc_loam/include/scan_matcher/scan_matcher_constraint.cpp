#include "scan_matcher_constraint.h"

namespace optimization{
    CeresScanMatcher3D::CeresScanMatcher3D(){
    }

    void CeresScanMatcher3D::Match(transform::Rigid3d* initial_pose_estimation,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_edge_in,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_surf_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_in_local,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud_in_local,
                   ceres::Solver::Summary *const summary,
                   bool& matcher_ok){
        double ceres_parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        Eigen::Map<Eigen::Quaterniond> q_w_curr =
                                    Eigen::Map<Eigen::Quaterniond>(ceres_parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr =
                                    Eigen::Map<Eigen::Vector3d>(ceres_parameters + 4);
        t_w_curr = initial_pose_estimation->translation();
        q_w_curr = initial_pose_estimation->rotation();

        ceres::LossFunction *loss_function =
            new ceres::HuberLoss(huberLoss_option);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(ceres_parameters, 7,
                                  new optimization::PoseSE3Parameterization());
        addEdgeCostFactor(pc_edge_in, corner_cloud_in_local,
                               problem, loss_function, matcher_ok, ceres_parameters,
                               t_w_curr, q_w_curr);
        addSurfCostFactor(pc_surf_in, surf_cloud_in_local,
                               problem, loss_function, matcher_ok, ceres_parameters,
                               t_w_curr, q_w_curr);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;

        ceres::Solve(options, &problem, summary);
        if(summary->termination_type == ceres::NO_CONVERGENCE)
            matcher_ok = false;

        *initial_pose_estimation = transform::Rigid3d(t_w_curr, q_w_curr);
    }

    void CeresScanMatcher3D::transformRangeData(pcl::PointXYZI const *const pi,
                                pcl::PointXYZI *const po,
                                Eigen::Map<Eigen::Vector3d>& t_w_curr,
                                Eigen::Map<Eigen::Quaterniond>& q_w_curr){
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void CeresScanMatcher3D::addEdgeCostFactor(
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
                           ceres::Problem& problem, ceres::LossFunction *loss_function,
                           bool& matcher_ok,
                           double* ceres_parameters,
                           Eigen::Map<Eigen::Vector3d>& t_w_curr,
                           Eigen::Map<Eigen::Quaterniond>& q_w_curr ){
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap(
                                 new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeEdgeMap->setInputCloud(map_in);
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;

        transformRangeData(&(pc_in->points[i]), &point_temp, t_w_curr, q_w_curr);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x,
                                    pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                problem.AddResidualBlock(optimization::CreateEdgeCostFunction(
                    curr_point, point_a, point_b),
                     loss_function, ceres_parameters);
                corner_num++;
            }
        }
    }
    if(corner_num<20){
        matcher_ok = false;
        //printf("not enough correct points");
    }

}

void CeresScanMatcher3D::addSurfCostFactor(
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
                 ceres::Problem& problem, ceres::LossFunction *loss_function,
                 bool& matcher_ok,
                 double* ceres_parameters,
                 Eigen::Map<Eigen::Vector3d>& t_w_curr,
                 Eigen::Map<Eigen::Quaterniond>& q_w_curr ){
    int surf_num=0;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap(
                                 new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap->setInputCloud(map_in);
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        transformRangeData(&(pc_in->points[i]), &point_temp, t_w_curr, q_w_curr);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {

            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z +
                                             negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y,
                                                     pc_in->points[i].z);
            if (planeValid)
            {
                problem.AddResidualBlock(
                    optimization::CreateSurfCostFunction(
                        curr_point, norm, negative_OA_dot_norm),
                         loss_function, ceres_parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        matcher_ok = false;
        //printf("not enough correct points");
    }

}
}
