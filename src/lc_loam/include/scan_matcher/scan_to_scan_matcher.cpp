#include "scan_to_scan_matcher.h"

namespace optimization{

    CeresScanMatcherScan::CeresScanMatcherScan(){}
    CeresScanMatcherScan::~CeresScanMatcherScan(){}

    void CeresScanMatcherScan::match(transform::Rigid3d *initial_pose_estimation,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_cur,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_cur,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_last,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_points_last,
                                     ceres::Solver::Summary *const summary)
    {
        double para_q[4] = {0, 0, 0, 1};
        double para_t[3] = {0, 0, 0};

        Eigen::Map<Eigen::Quaterniond> q_constraint(para_q);
        Eigen::Map<Eigen::Vector3d> t_constraint(para_t);
        t_constraint = initial_pose_estimation->translation();
        q_constraint = initial_pose_estimation->rotation();

        // ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);
        int corner_correspondence = 0;
        int plane_correspondence = 0;
        addEdgeFactor(edge_points_cur, edge_points_last,
                      problem, loss_function,
                      para_q, para_t, &corner_correspondence);
        addSurfFactor(surf_points_cur, surf_points_last,
                      problem, loss_function,
                      para_q, para_t, &plane_correspondence);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solve(options, &problem, summary);

        *initial_pose_estimation = transform::Rigid3d(t_constraint, q_constraint);
    }

 void CeresScanMatcherScan::addEdgeFactor(
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_cur,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_points_last,
                         ceres::Problem &problem, ceres::LossFunction *loss_function,
                         double* para_q,
                         double* para_t,
                         int* corner_correspondence){
        *corner_correspondence = 0;
        int cornerPointsSharpNum = edge_points_cur->points.size();
        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeLast(
                           new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtreeEdgeLast->setInputCloud(edge_points_last);
        for (int i = 0; i < cornerPointsSharpNum; ++i)
        {
            kdtreeEdgeLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closestPointInd = pointSearchInd[0];
                int closestPointScanID = int(
                    edge_points_last->points[closestPointInd].intensity);

                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)edge_points_last->points.size(); ++j)
                {
                 // if in the same scan line, continue
                    if (int(edge_points_last->points[j].intensity) <= closestPointScanID)
                      continue;

                  // if not in nearby scans, end the loop
                    if (int(edge_points_last->points[j].intensity) >
                                        (closestPointScanID + NEARBY_SCAN))
                      break;
                    double pointSqDis = (edge_points_last->points[j].x - pointSel.x) *
                                         (edge_points_last->points[j].x - pointSel.x) +
                                         (edge_points_last->points[j].y - pointSel.y) *
                                         (edge_points_last->points[j].y - pointSel.y) +
                                         (edge_points_last->points[j].z - pointSel.z) *
                                         (edge_points_last->points[j].z - pointSel.z);

                     if (pointSqDis < minPointSqDis2)
                        {
                           // find nearer point
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                }
                            // search in the direction of decreasing scan line
                 for (int j = closestPointInd - 1; j >= 0; --j)
                {
                  // if in the same scan line, continue
                  if (int(edge_points_last->points[j].intensity) >= closestPointScanID)
                     continue;

                  // if not in nearby scans, end the loop
                  if (int(edge_points_last->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                     break;
                  double pointSqDis = (edge_points_last->points[j].x - pointSel.x) *
                                       (edge_points_last->points[j].x - pointSel.x) +
                                       (edge_points_last->points[j].y - pointSel.y) *
                                       (edge_points_last->points[j].y - pointSel.y) +
                                       (edge_points_last->points[j].z - pointSel.z) *
                                       (edge_points_last->points[j].z - pointSel.z);

                  if (pointSqDis < minPointSqDis2)
                      {
                           // find nearer point
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                   }
                }
                 if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                 {
                     Eigen::Vector3d curr_point(edge_points_cur->points[i].x,
                                                edge_points_cur->points[i].y,
                                                edge_points_cur->points[i].z);
                     Eigen::Vector3d last_point_a(edge_points_last->points[closestPointInd].x,
                                                edge_points_last->points[closestPointInd].y,
                                                edge_points_last->points[closestPointInd].z);
                     Eigen::Vector3d last_point_b(edge_points_last->points[minPointInd2].x,
                                                  edge_points_last->points[minPointInd2].y,
                                                  edge_points_last->points[minPointInd2].z);
                     double s = 1.0;
                     ceres::CostFunction *cost_function = createEdge(curr_point, last_point_a, last_point_b, s);
                     problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                     *corner_correspondence += 1;
                     }
        }

   }
 void CeresScanMatcherScan::addSurfFactor(
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_cur,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_points_last,
                         ceres::Problem &problem, ceres::LossFunction *loss_function,
                         double* para_q,
                         double* para_t,
                         int* plane_correspondence){
       int surfPointsFlatNum = surf_points_last->points.size();
       *plane_correspondence = 0;

       pcl::PointXYZI pointSel;
       std::vector<int> pointSearchInd;
       std::vector<float> pointSearchSqDis;

       pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(
                           new pcl::KdTreeFLANN<pcl::PointXYZI>());
       kdtreeSurfLast->setInputCloud(surf_points_last);
       for (int i = 0; i < surfPointsFlatNum; ++i){
          kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD){
              closestPointInd = pointSearchInd[0];

              // get closest point's scan ID
              int closestPointScanID = int(surf_points_last->points[closestPointInd].intensity);
              double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

              // search in the direction of increasing scan line
              for (int j = closestPointInd + 1; j < (int)surf_points_last->points.size(); ++j)
              {
                  // if not in nearby scans, end the loop
                  if (int(surf_points_last->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                      break;

                  double pointSqDis = (surf_points_last->points[j].x - pointSel.x) *
                                      (surf_points_last->points[j].x - pointSel.x) +
                                      (surf_points_last->points[j].y - pointSel.y) *
                                      (surf_points_last->points[j].y - pointSel.y) +
                                      (surf_points_last->points[j].z - pointSel.z) *
                                      (surf_points_last->points[j].z - pointSel.z);

                  // if in the same or lower scan line
                  if (int(surf_points_last->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                  {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                  }
                  // if in the higher scan line
                  else if (int(surf_points_last->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                  {
                      minPointSqDis3 = pointSqDis;
                      minPointInd3 = j;
                  }
                  }

                  // search in the direction of decreasing scan line
                  for (int j = closestPointInd - 1; j >= 0; --j){
                     // if not in nearby scans, end the loop
                     if (int(surf_points_last->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                     double pointSqDis = (surf_points_last->points[j].x - pointSel.x) *
                                         (surf_points_last->points[j].x - pointSel.x) +
                                         (surf_points_last->points[j].y - pointSel.y) *
                                         (surf_points_last->points[j].y - pointSel.y) +
                                         (surf_points_last->points[j].z - pointSel.z) *
                                         (surf_points_last->points[j].z - pointSel.z);

                     // if in the same or higher scan line
                     if (int(surf_points_last->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                     {
                         minPointSqDis2 = pointSqDis;
                         minPointInd2 = j;
                     }
                     else if (int(surf_points_last->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                     {
                         // find nearer point
                         minPointSqDis3 = pointSqDis;
                         minPointInd3 = j;
                     }
                     }

                     if (minPointInd2 >= 0 && minPointInd3 >= 0)
                     {
                        Eigen::Vector3d curr_point(surf_points_cur->points[i].x,
                                                   surf_points_cur->points[i].y,
                                                   surf_points_cur->points[i].z);
                        Eigen::Vector3d last_point_a(surf_points_last->points[closestPointInd].x,
                                                    surf_points_last->points[closestPointInd].y,
                                                    surf_points_last->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(surf_points_last->points[minPointInd2].x,
                                                    surf_points_last->points[minPointInd2].y,
                                                    surf_points_last->points[minPointInd2].z);
                        Eigen::Vector3d last_point_c(surf_points_last->points[minPointInd3].x,
                                                     surf_points_last->points[minPointInd3].y,
                                                     surf_points_last->points[minPointInd3].z);
                        double s = 1.0;
                        ceres::CostFunction *cost_function = createSurf(curr_point, last_point_a, last_point_b, last_point_c, s);
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        *plane_correspondence += 1;
                     }
                }
          }

 }


}