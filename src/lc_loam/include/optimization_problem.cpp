#include "optimization_problem.h"

namespace optimization{

    OptimizationProblem::~OptimizationProblem(){}

    OptimizationProblem::OptimizationProblem(){
        std::cout << "--------optimization111------" << std::endl;
    }

    OptimizationProblem::OptimizationProblem(bool fix_z)
    : fix_z_in_3d(fix_z){
        std::cout << "--------optimization------" << std::endl;
        std::cout << fix_z_in_3d << std::endl;
    }

    void OptimizationProblem::AddSubmap(int trajectory_id,
                           const transform::Rigid3d &global_submap_pose){
        submap_data_.Append(trajectory_id, SubmapSpec{global_submap_pose});
    }

    void OptimizationProblem::InsertSubmap(const mapping::SubmapId& submap_id,
                           const transform::Rigid3d &global_submap_pose){
        submap_data_.Insert(submap_id, SubmapSpec{global_submap_pose});
    }

    void OptimizationProblem::TrimSubmap(const mapping::SubmapId& submap_id){
        submap_data_.Trim(submap_id);
    }

    void OptimizationProblem::AddTrajectoryNode(int trajectory_id,
                                  const NodeSpec& node_data){
        node_data_.Append(trajectory_id, node_data);
    }

    void OptimizationProblem::InsertTrajectoryNode(const mapping::NodeId& node_id, /*in node_id*/
                                   const NodeSpec &node_data){
        node_data_.Insert(node_id, node_data);
    }


    void OptimizationProblem::Solve(
             const std::vector<mapping::PoseGraphInterface::Constraint> &constraints,
             const std::map<int, mapping::PoseGraphInterface::TrajectoryState>&
             trajectories_state){
        if(node_data_.empty()){
            return;
        }
        std::set<int> frozen_trajectories;
        for(const auto& it : trajectories_state){
            if(it.second == mapping::PoseGraphInterface::TrajectoryState::FROZEN){
                frozen_trajectories.insert(it.first);
            }
        }

        const auto translation_parameterization =
        [this]() -> std::unique_ptr<ceres::LocalParameterization> {
        return fix_z_in_3d
               ? absl::make_unique<ceres::SubsetParameterization>(
                     3, std::vector<int>{2})
               : nullptr;
        };

        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        CHECK(!submap_data_.empty());
        mapping::MapById<mapping::SubmapId, CeresPose> C_submaps;
        mapping::MapById<mapping::NodeId, CeresPose> C_nodes;
        bool first_submap = true;
        for(const auto& submap_id_data : submap_data_){
            const bool frozen =
                frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
            if(first_submap){
            first_submap = false;
            C_submaps.Insert(submap_id_data.id,
                             CeresPose(submap_id_data.data.global_pose,
                             nullptr,
                             nullptr,
                             &problem));
            problem.SetParameterBlockConstant(
                 C_submaps.at(submap_id_data.id).translation());
            problem.SetParameterBlockConstant(
                 C_submaps.at(submap_id_data.id).rotation());
            }
            else{
                C_submaps.Insert(submap_id_data.id,
                    CeresPose(submap_id_data.data.global_pose,
                              translation_parameterization(),
                              absl::make_unique<ceres::QuaternionParameterization>(),
                              &problem));
            }
            if(frozen){
                problem.SetParameterBlockConstant(
                    C_submaps.at(submap_id_data.id).rotation());
                problem.SetParameterBlockConstant(
                    C_submaps.at(submap_id_data.id).translation());
            }
        }
        for(const auto& node_id_data : node_data_){
            const bool frozen =
                frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
            C_nodes.Insert(node_id_data.id,
                           CeresPose(node_id_data.data.global_pose_3d, translation_parameterization(),
                                     absl::make_unique<ceres::QuaternionParameterization>(),
                                     &problem));
            if(frozen){
                problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).rotation());
                problem.SetParameterBlockConstant(
                    C_nodes.at(node_id_data.id).translation());
            }
        }
        for(const Constraint& constraint : constraints){
            problem.AddResidualBlock(
                SpaCostFunction3D::CreateAutoDiffCostFunction(constraint.pose),
                constraint.tag == Constraint::INTER_SUBMAP ?
                new ceres::HuberLoss(0.1) : nullptr,
                C_submaps.at(constraint.submap_id).rotation(),
                C_submaps.at(constraint.submap_id).translation(),
                C_nodes.at(constraint.node_id).rotation(),
                C_nodes.at(constraint.node_id).translation());
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 50;
        options.num_threads = 4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // store the result
        for(const auto& C_submap_id_data : C_submaps){
            submap_data_.at(C_submap_id_data.id).global_pose =
                C_submap_id_data.data.ToRigid();
        }
        for(const auto& C_node_id_data : C_nodes){
            node_data_.at(C_node_id_data.id).global_pose_3d =
                C_node_id_data.data.ToRigid();
            // std::cout << "C_node_id_data.id: " <<
            // C_node_id_data.id << node_data_.at(C_node_id_data.id).global_pose_3d
            //           << std::endl;
        }
        std::cout << "optimization finished!" << std::endl;
    }

}//
