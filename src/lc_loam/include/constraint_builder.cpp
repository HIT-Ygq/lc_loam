#include "constraint_builder.h"

namespace optimization{

     ConstraintBuilder::ConstraintBuilder(
         common::ThreadPoolInterface* thread_pool):
         thread_pool_(thread_pool),
         finish_node_task_(absl::make_unique<common::Task>()),
         when_done_task_(absl::make_unique<common::Task>()){
         }

    ConstraintBuilder::ConstraintBuilder(
        common::ThreadPoolInterface* thread_pool,
        const common::LoopClosureOptions& options):
        thread_pool_(thread_pool),
        finish_node_task_(absl::make_unique<common::Task>()),
        when_done_task_(absl::make_unique<common::Task>()),
        num_range_data_(options.num_range_data),
        loop_closure_translation_weight(options.loop_closure_translation_weight),
        loop_closure_rotation_weight(options.loop_closure_rotation_weight),
        submap_threshold(options.submap_threshold),
        node_id_threshold(options.node_id_threshold),
        scan_to_scan_min(options.scan_to_scan_min),
        distance_threshold(options.distance_threshold),
        num_of_submap_threshold(options.num_of_submap_threshold)
        {
        }

    ConstraintBuilder::~ConstraintBuilder() {
        absl::MutexLock locker(&mutex_);
        CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
        CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
        CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
        CHECK_EQ(num_started_nodes_, num_finished_nodes_);
        CHECK(when_done_ == nullptr);
    }

    void ConstraintBuilder::MaybeAddConstraint(
        const mapping::SubmapId& submap_id, const Submap* const submap,
        const mapping::NodeId& node_id,
        const mapping::PoseGraphData& data,
        const mapping::TrajectoryNode::Data* const constant_data,
        transform::Rigid3d global_pose,
        const transform::Rigid3d& global_node_pose,
        const transform::Rigid3d& global_submap_pose,
        bool& loop_closure_flag
    ){
         if ((global_node_pose.translation() - global_submap_pose.translation())
          .norm() > distance_threshold) {
          return;
        }
        absl::MutexLock locker(&mutex_);
        if(when_done_){
             LOG(WARNING)
                 << "MaybeAddConstraint was called while WhenDone was scheduled.";
        }
        constraints_.emplace_back();
        auto* const constraint = &constraints_.back();
        auto constraint_task = absl::make_unique<common::Task>();
        constraint_task->SetWorkItem([=, &loop_closure_flag]() {
        ComputeConstraint(submap_id, node_id, data, false, /* match_full_submap */
                      constant_data, global_pose,
                      submap, constraint, loop_closure_flag);
        });
        auto constraint_task_handle =
        thread_pool_->Schedule(std::move(constraint_task));
        finish_node_task_->AddDependency(constraint_task_handle);
    }
   // todo:
   void ConstraintBuilder::ComputeConstraint(const mapping::SubmapId& submap_id,
                         const mapping::NodeId& node_id,
                         const mapping::PoseGraphData& data,
                         bool match_full_map,
                         const mapping::TrajectoryNode::Data* const constant_data,
                         transform::Rigid3d global_pose,
                         const Submap* submap,
                         std::unique_ptr<Constraint>* constraint,
                         bool& loop_closure_flag){
       if (match_full_map)
       {
           LOG(WARNING) << "match_full_map: i'm so soory to tell i can't do it";
           return;
       }
         // constant_data当前帧，submap是地图,也就是做constant_data和submap的匹配
         // change this config when run in kejiyuan   50 or 20

        //  if(node_id.node_index > 12000 &&  node_id.node_index < 14999 &&
        //    submap_id.submap_index <= 33){
        //        return;
        //  }
         if(submap_id.submap_index <= submap_threshold && node_id.node_index < node_id_threshold){
             if((submap->center_pose().translation() -
                constant_data->local_pose.translation()).norm() > distance_threshold)
                return;
         }
         else{
             if((submap->center_pose().translation() -
                global_pose.translation()).norm() > distance_threshold)
                return;
         }
        //  if ((submap_id.submap_index > 50 || submap_id.submap_index <= 20) && (submap->center_pose().translation() -
        //         global_pose.translation()).norm() > distance_threshold)
        //      return;
        //  if (submap_id.submap_index <= 50 && submap_id.submap_index > 20 && (submap->center_pose().translation() -
        //         constant_data->local_pose.translation()).norm() > distance_threshold)
        //      return;
         // change this config when run in kejiyuan  100 or 2
         if(node_id.node_index / num_range_data_ - submap_id.submap_index
            <= num_of_submap_threshold)
             return;
         double min_distance = 10000;
         int min_scan_id = findNearestScan(submap_id, node_id, data, constant_data,
                                  global_pose, num_range_data_, min_distance,
                                  submap_threshold,  node_id_threshold);
         if ( min_distance > scan_to_scan_min )
            return;

          bool hasConverged = true;
        //   transform::Rigid3d transform_estimate =
        //                             ndtScanMatcher(data, constant_data, submap,
        //                             min_scan_id, &hasConverged);
          transform::Rigid3d transform_scan_to_scan =
              icpScanMatch(data, constant_data, min_scan_id, hasConverged);
          if(!hasConverged)
              return;
        //   ceres::Solver::Summary unused_summary_1;
        //   scan_to_scan_matcher_.match(&transform_scan_to_scan,
        //                               constant_data->range_data_raw.edge_data_in_local,
        //                               constant_data->range_data_raw.surf_data_in_local,
        //                               data.trajectory_nodes.at(
        //                                                        mapping::NodeId{0, min_scan_id})
        //                                   .constant_data->range_data_raw.edge_data_in_local,
        //                               data.trajectory_nodes.at(
        //                                                        mapping::NodeId{0, min_scan_id})
        //                                   .constant_data->range_data_raw.surf_data_in_local,
        //                               &unused_summary_1);

        //   transform::Rigid3d transform_estimate_scan =
        //       submap->local_pose().inverse() *
        //       data.trajectory_nodes.at(
        //                                mapping::NodeId{0, min_scan_id})
        //           .constant_data->local_pose *
        //       transform_scan_to_scan;

        //   std::cout << "ceres-constraint: "
        //             << "submap_id: "
        //             << submap_id.submap_index << " node_id: " << node_id.node_index << transform_estimate_scan << std::endl;

          transform::Rigid3d transform_estimate = data.trajectory_nodes.at(
                                                                           mapping::NodeId{0, min_scan_id})
                                                      .constant_data->local_pose *
                                                  transform_scan_to_scan;
         ceres::Solver::Summary unused_summary;

         bool matcher_ok = true;
         for (int iter = 0; iter != optimization_count; ++iter)
         {
             ceres_scan_matcher_.Match(&transform_estimate,
                                       constant_data->range_data_raw.edge_data_in_local,
                                       constant_data->range_data_raw.surf_data_in_local,
                                       submap->conerMap(),
                                       submap->surfMap(),
                                       &unused_summary,
                                       matcher_ok);
        }
           if (!matcher_ok)
                return;
        //  std::cout << "ceres-constraint-map: " << "submap_id: "
        //  << submap_id.submap_index << " node_id: " << node_id.node_index <<
        //  submap->local_pose().inverse() * transform_estimate << std::endl;

         LOG(INFO) << "submap_id: " << submap_id.submap_index <<
         " node_id: " << node_id.node_index << " constraint construction!";
         loop_closure_flag = true;
         constraint->reset(new Constraint{
               submap_id,
               node_id,
               {submap->local_pose().inverse() * transform_estimate,
                loop_closure_translation_weight,
                loop_closure_rotation_weight},
               Constraint::INTER_SUBMAP});
   }

    // todo::
    void ConstraintBuilder::RunWhenDoneCallback(){

    Result result;
    std::unique_ptr<std::function<void(const Result&)>> callback;
   {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);

    // 将计算完的约束进行保存
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }

    // 这些约束已经保存过了, 就可以删掉了
    constraints_.clear();

    callback = std::move(when_done_);
    when_done_.reset();
    //kQueueLengthMetric->Set(constraints_.size());
  }
  // 执行回调函数 HandleWorkQueue
  (*callback)(result);

}

   // todo:
   int ConstraintBuilder::GetNumFinishedNodes(){
        absl::MutexLock locker(&mutex_);
        return num_finished_nodes_;
    }
   // todo:
   void ConstraintBuilder::NotifyEndOfNode(){
        absl::MutexLock locker(&mutex_);
        CHECK(finish_node_task_ != nullptr);

        // 生成个任务: 将num_finished_nodes_自加, 记录完成约束计算节点的总个数
        finish_node_task_->SetWorkItem([this] {
        absl::MutexLock locker(&mutex_);
        ++num_finished_nodes_;
        });

        // 将这个任务传入线程池中等待执行, 由于之前添加了依赖,
        // 所以finish_node_task_一定会比计算约束更晚完成
        auto finish_node_task_handle =
        thread_pool_->Schedule(std::move(finish_node_task_));

        // move之后finish_node_task_就没有指向的地址了, 所以这里要重新初始化
        finish_node_task_ = absl::make_unique<common::Task>();
        // 设置when_done_task_依赖finish_node_task_handle
        when_done_task_->AddDependency(finish_node_task_handle);
        ++num_started_nodes_;
    }


    void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)>& callback) {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ == nullptr);

    // TODO(gaschler): Consider using just std::function, it can also be empty.
    // 将回调函数赋值给when_done_
    when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
    CHECK(when_done_task_ != nullptr);

    // 生成 执行when_done_的任务
    when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
    // 将任务放入线程池中等待执行
    thread_pool_->Schedule(std::move(when_done_task_));

    // when_done_task_的重新初始化
    when_done_task_ = absl::make_unique<common::Task>();
}

}
