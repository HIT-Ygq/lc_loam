#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

namespace optimization{
class LidarEdgeFactor
{
  public:
      LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                      Eigen::Vector3d last_point_b_, double s_);

      template <typename T>
      bool operator()(const T *q, const T *t, T *residual) const
      {

          Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
          Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
          Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

          // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
          Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
          Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
          q_last_curr = q_identity.slerp(T(s), q_last_curr);
          Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

          Eigen::Matrix<T, 3, 1> lp;
          lp = q_last_curr * cp + t_last_curr;

          Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
          Eigen::Matrix<T, 3, 1> de = lpa - lpb;

          residual[0] = nu.x() / de.norm();
          residual[1] = nu.y() / de.norm();
          residual[2] = nu.z() / de.norm();

          return true;
	}
private:
	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

ceres::CostFunction *createEdge(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                const Eigen::Vector3d last_point_b_, const double s_);

class LidarPlaneFactor
{
    public:
        LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                         Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_,
                         double s_);

        template <typename T>
        bool operator()(const T *q, const T *t, T *residual) const
        {

            Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
            Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
            // Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
            // Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
            Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

            // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
            Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
            Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
            q_last_curr = q_identity.slerp(T(s), q_last_curr);
            Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

            Eigen::Matrix<T, 3, 1> lp;
            lp = q_last_curr * cp + t_last_curr;

            residual[0] = (lp - lpj).dot(ljm);

            return true;
	}

private:
    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

ceres::CostFunction *createSurf(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
								const Eigen::Vector3d last_point_l_,
								const Eigen::Vector3d last_point_m_,
								const double s_);

}