#pragma once

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "ceres/ceres.h"
#include "ceres/jet.h"

//#include "pose_graph.h"
#include "common/transform.h"
#include "scan_matcher/cost_helpers_impl.h"

namespace optimization {

class SpaCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const mapping::PoseGraphInterface::Constraint::Pose& pose) {
    return new ceres::AutoDiffCostFunction<
        SpaCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */,
        3 /* translation variables */>(new SpaCostFunction3D(pose));
  }

  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(pose_.zbar_ij, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation),
        pose_.translation_weight, pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit SpaCostFunction3D(const mapping::PoseGraphInterface::Constraint::Pose& pose)
      : pose_(pose) {}

  const mapping::PoseGraphInterface::Constraint::Pose pose_;
};

}  // namespace optimization