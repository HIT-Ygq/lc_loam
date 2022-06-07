#pragma once

#include "Eigen/Core"
#include "common/transform.h"

namespace optimization {

/**
 * @brief 2d 根据SPA论文里的公式求残差
 *
 * 计算残差：
 * T12 = T1.inverse() * T2
 * [R1.inverse * R2,  R1.inverse * (t2 -t1)]
 * [0              ,  1                    ]
 *
 * @param[in] relative_pose
 * @param[in] start
 * @param[in] end
 * @return std::array<T, 3>
 */

/**
 * @brief 根据SPA论文里的公式求6维度的残差
 *
 * @param[in] relative_pose
 * @param[in] start_rotation
 * @param[in] start_translation
 * @param[in] end_rotation
 * @param[in] end_translation
 * @return std::array<T, 6>
 */
template <typename T>
static std::array<T, 6> ComputeUnscaledError(
    const transform::Rigid3d& relative_pose, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation) {
  const Eigen::Quaternion<T> R_i_inverse(start_rotation[0], -start_rotation[1],
                                         -start_rotation[2],
                                         -start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> delta(end_translation[0] - start_translation[0],
                                     end_translation[1] - start_translation[1],
                                     end_translation[2] - start_translation[2]);
  // start到end的平移
  const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

  // start到end的旋转 四元数的转置就是逆
  const Eigen::Quaternion<T> h_rotation_inverse =
      Eigen::Quaternion<T>(end_rotation[0], -end_rotation[1], -end_rotation[2],
                           -end_rotation[3]) *
      Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                           start_rotation[2], start_rotation[3]);

  // 计算2个旋转间的差值
  const Eigen::Matrix<T, 3, 1> angle_axis_difference =
      transform::RotationQuaternionToAngleAxisVector(
          h_rotation_inverse * relative_pose.rotation().cast<T>());

  return {{T(relative_pose.translation().x()) - h_translation[0],
           T(relative_pose.translation().y()) - h_translation[1],
           T(relative_pose.translation().z()) - h_translation[2],
           angle_axis_difference[0],
           angle_axis_difference[1],
           angle_axis_difference[2]}};
}

// 3d 为残差添加权重
template <typename T>
std::array<T, 6> ScaleError(const std::array<T, 6>& error,
                            double translation_weight, double rotation_weight) {
  // clang-format off
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * translation_weight,
      error[3] * rotation_weight,
      error[4] * rotation_weight,
      error[5] * rotation_weight
  }};
  // clang-format on
}

// Eigen implementation of slerp is not compatible with Ceres on all supported
// platforms. Our own implementation is used instead.
// slerp 的Eigen实现与所有支持平台上的 Ceres 不兼容, 所以自己实现
template <typename T>
std::array<T, 4> SlerpQuaternions(const T* const start, const T* const end,
                                  double factor) {
  // Angle 'theta' is the half-angle "between" quaternions. It can be computed
  // as the arccosine of their dot product.
  const T cos_theta = start[0] * end[0] + start[1] * end[1] +
                      start[2] * end[2] + start[3] * end[3];
  // Avoid using ::abs which would cast to integer.
  const T abs_cos_theta = ceres::abs(cos_theta);
  // If numerical error brings 'cos_theta' outside [-1 + epsilon, 1 - epsilon]
  // interval, then the quaternions are likely to be collinear.
  T prev_scale(1. - factor);
  T next_scale(factor);
  if (abs_cos_theta < T(1. - 1e-5)) {
    const T theta = acos(abs_cos_theta);
    const T sin_theta = sin(theta);
    prev_scale = sin((1. - factor) * theta) / sin_theta;
    next_scale = sin(factor * theta) / sin_theta;
  }
  if (cos_theta < T(0.)) {
    next_scale = -next_scale;
  }
  return {{prev_scale * start[0] + next_scale * end[0],
           prev_scale * start[1] + next_scale * end[1],
           prev_scale * start[2] + next_scale * end[2],
           prev_scale * start[3] + next_scale * end[3]}};
}

template <typename T>
std::tuple<std::array<T, 4> /* rotation */, std::array<T, 3> /* translation */>
InterpolateNodes3D(const T* const prev_node_rotation,
                   const T* const prev_node_translation,
                   const T* const next_node_rotation,
                   const T* const next_node_translation,
                   const double interpolation_parameter) {
  return std::make_tuple(
      SlerpQuaternions(prev_node_rotation, next_node_rotation,
                       interpolation_parameter),
      std::array<T, 3>{
          {prev_node_translation[0] +
               interpolation_parameter *
                   (next_node_translation[0] - prev_node_translation[0]),
           prev_node_translation[1] +
               interpolation_parameter *
                   (next_node_translation[1] - prev_node_translation[1]),
           prev_node_translation[2] +
               interpolation_parameter *
                   (next_node_translation[2] - prev_node_translation[2])}});
}


}  // namespace optimization