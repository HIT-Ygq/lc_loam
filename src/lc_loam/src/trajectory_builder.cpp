
#include "trajectory_builder.h"

LocalTrajectoryBuilder::LocalTrajectoryBuilder():thread_pool_(4){
    pose_graph_ = absl::make_unique<mapping::PoseGraph>(
        absl::make_unique<optimization::OptimizationProblem>(),
        &thread_pool_);
}


LocalTrajectoryBuilder::LocalTrajectoryBuilder(const common::NodeOptions& options)
     : scan_period(options.odom_options.scan_period),
     speed_filter(options.odom_options.speed_filter),
     point_cloud_map_(new pcl::PointCloud<pcl::PointXYZI>()),
     fullMap(new pcl::PointCloud<pcl::PointXYZI>()),
     thread_pool_(options.odom_options.num_thread_pool)
     {
     pose_graph_ = absl::make_unique<mapping::PoseGraph>(
        absl::make_unique<optimization::OptimizationProblem>(options.pose_graph_options.fix_z_in_3d),
        &thread_pool_, options.pose_graph_options);

    downSizeFilterEdge.setLeafSize(options.odom_options.map_resolution_edge,
                            options.odom_options.map_resolution_edge,
                            options.odom_options.map_resolution_edge);
    downSizeFilterSurf.setLeafSize(options.odom_options.map_resolution_surf,
                       options.odom_options.map_resolution_surf,
                       options.odom_options.map_resolution_surf);
    downSizeFilterFull.setLeafSize(options.odom_options.map_resolution_edge,
                            options.odom_options.map_resolution_edge,
                            options.odom_options.map_resolution_edge);
    downSizeFullMap.setLeafSize(options.odom_options.map_resolution_edge,
                      options.odom_options.map_resolution_edge,
                       options.odom_options.map_resolution_edge);

    optimization_count = 12;

    subEdgeLaserCloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge",
       100, &LocalTrajectoryBuilder::velodyneEdgeHandler, this);
    subSurfLaserCloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf",
       100, &LocalTrajectoryBuilder::velodyneSurfHandler, this);
    filterPointCloud =
        nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered",
        100, &LocalTrajectoryBuilder::velodyneFullHandle, this);

    local_path_publisher = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubSubmap_corner = nh.advertise<sensor_msgs::PointCloud2>("/submap_corner", 100);
    pubSubmap_surf = nh.advertise<sensor_msgs::PointCloud2>("/submap_surf", 100);
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    pubLastMap = nh.advertise<sensor_msgs::PointCloud2>("/map_1", 100);
    global_path_publisher = nh.advertise<nav_msgs::Path>("/global_odom", 10);
    last_map_ = nh.advertiseService("/last_map",
                      &LocalTrajectoryBuilder::last_map, this);
    last_optimization = nh.advertiseService("/optimization",
                      &LocalTrajectoryBuilder::lastOptimization, this);
    save_map = nh.advertiseService("/save_map",
                      &LocalTrajectoryBuilder::savePCD, this);
    save_pose = nh.advertiseService("/save_global_pose",
                      &LocalTrajectoryBuilder::save_trajectory, this);

    constraint_list_publisher_ =
          nh.advertise<visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);

    wall_timers_.push_back(nh.createWallTimer(
        ros::WallDuration(kConstraintPublishPeriodSec),
        &LocalTrajectoryBuilder::PublishAfterOptimization, this));

    wall_timers_.push_back(nh.createWallTimer(
        ros::WallDuration(kConstraintPublishPeriodSec),
        &LocalTrajectoryBuilder::PublishConstraintList, this));

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();

    active_submaps_.set_num_range_data(options.odom_options.num_range_data);
    active_submaps_.set_edge_resolution(options.odom_options.map_resolution_edge);
    active_submaps_.set_surf_resolution(options.odom_options.map_resolution_surf);
}


void LocalTrajectoryBuilder::velodyneSurfHandler(
                  const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void LocalTrajectoryBuilder::velodyneEdgeHandler(
                 const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void LocalTrajectoryBuilder::velodyneFullHandle(
                const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    mutex_lock.lock();
    pointCloudFullBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void LocalTrajectoryBuilder::odom_estimation(){
    while(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){
            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() &&
                (pointCloudSurfBuf.front()->header.stamp.toSec() <
                    pointCloudEdgeBuf.front()->header.stamp.toSec()- 0.5 *
                      scan_period)){
                pointCloudSurfBuf.pop();
                LOG(WARNING) << "time stamp unaligned with extra point cloud,"
                        "pls check your data-- >odom correction ";
                mutex_lock.unlock();
                continue;
            }

            if(!pointCloudEdgeBuf.empty() &&
              (pointCloudEdgeBuf.front()->header.stamp.toSec() <
                 pointCloudSurfBuf.front()->header.stamp.toSec()- 0.5 *
                   scan_period)){
                pointCloudEdgeBuf.pop();
                LOG(WARNING) << "time stamp unaligned with extra point cloud,"
                                "pls check your data --> odom correction";
                mutex_lock.unlock();
                continue;
            }
            //if time aligned
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(
                                              new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(
                                              new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_full_in(
                                              new pcl::PointCloud<pcl::PointXYZI>());

            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl::fromROSMsg(*pointCloudFullBuf.front(), *pointcloud_full_in);

            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            pointCloudFullBuf.pop();
            mutex_lock.unlock();

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            addRangeData(pointcloud_edge_in, pointcloud_surf_in,
            pointcloud_full_in);

            // 计时并输出时间
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time += time_temp;
            // std::ofstream fout("/home/xcy/experiment/time/time_our.txt", std::ios::app);
            // fout << total_time/total_frame << std::endl;
            // fout.close();
            // 发布tf变换和里程计
            publishRosTf(pointcloud_time);

            std::shared_ptr<const Submap> first_active_submap = getFirstSubmap();
            pcl::PointCloud<pcl::PointXYZI>::Ptr first_active_submap_corner =
                                                    first_active_submap->conerMap();
            publishPointCloud(first_active_submap_corner,
                                         pointcloud_time, pubSubmap_corner);

            pcl::PointCloud<pcl::PointXYZI>::Ptr first_active_submap_surf
                                                   = first_active_submap->surfMap();
            publishPointCloud(first_active_submap_surf,
                                         pointcloud_time, pubSubmap_surf);

            //publishMap(pointcloud_time, pubMap);
            // loop_closure_test();
        //     if(getFirstSubmap()->insertion_finished()){
		//          fout.open("/home/xcy/txt/submap_pose.txt", std::ios::app);
        //            fout << getFirstSubmap()->local_pose()
        //              << std::endl;
        //             fout.close();
        //    }
        }
}


void LocalTrajectoryBuilder::addRangeData(
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_in){
    counter++;
    full_in->clear();
    if(optimization_count > 2)
        optimization_count--;

    //Eigen::Isometry3d velocity = last_odom.inverse() * odom;
    // velocity.translation().y() = 0;
    // velocity.translation().z() = 0;
    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    transform::Rigid3d pose_estimate(odom.translation(),
                        Eigen::Quaterniond(odom.rotation()));
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(
                                         new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(
                                         new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledFullCloud(
                                         new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in, downsampledEdgeCloud,
                      surf_in, downsampledSurfCloud,
                      full_in, downsampledFullCloud);
    if(active_submaps_.submaps().empty())
    {
        std::vector<std::shared_ptr<const Submap>> insertion_submaps =
                      active_submaps_.InsertRangeData(downsampledEdgeCloud,
                                                      downsampledSurfCloud,
                                                      pose_estimate);
    }
    std::shared_ptr<const Submap> matching_submap =
        active_submaps_.submaps().front();
    bool matcher_ok = true;
    if (matching_submap->conerMap()->points.size() > 10 &&
        matching_submap->surfMap()->points.size() > 50)
    {
        ceres_scan_matcher.setKdtree(matching_submap.get());
        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            ceres::Solver::Summary summary;
            ceres_scan_matcher.Match(&pose_estimate, downsampledEdgeCloud,
             downsampledSurfCloud, matching_submap.get(),
             &summary, matcher_ok);

             if(!matcher_ok){
                 LOG(WARNING) << "not enough points!!";
             }
        }
        //std::cout << pose_estimate << std::endl;
    }
    else{
        LOG(WARNING) << "not enough points in map to associate, map error";
    }

    //kejiyuan
    Eigen::Vector3d trans_k(pose_estimate.translation().x(), pose_estimate.translation().y(), 0);
    Eigen::Quaterniond rotation_k(pose_estimate.rotation().w(), 0, 0, pose_estimate.rotation().z());
    rotation_k.normalize();
    transform::Rigid3d pose_k(trans_k, rotation_k);
    pose_estimate = pose_k;

    odom = Eigen::Isometry3d::Identity();
    odom.linear() = pose_estimate.rotation().toRotationMatrix();
    odom.translation() = pose_estimate.translation();

    // std::ofstream fout1("/home/xcy/experiment/skip/kitti05.txt", std::ios::app);
    if((odom.translation() - last_odom.translation()).norm() < speed_filter)
    {
        // fout1 << "------------count: " << counter << " ---------------";
        // fout1 << std::endl;
        return;
    }
    // fout1.close();
    // 将原点位于local坐标系原点处的点云 变换成 原点位于匹配位姿处的点云。
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_in_local(
                                              new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_points_in_local(
                                              new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_points_in_local(
                                              new pcl::PointCloud<pcl::PointXYZI>());
    edge_points_in_local->reserve(downsampledEdgeCloud->points.size());
    surf_points_in_local->reserve(downsampledSurfCloud->points.size());
    full_points_in_local->reserve(downsampledFullCloud->points.size());
    transform::tranformRageData(downsampledEdgeCloud,
                                edge_points_in_local,
                                pose_estimate);
    transform::tranformRageData(downsampledSurfCloud,
                                surf_points_in_local,
                                pose_estimate);
    // 将原点位于匹配位姿处的点云插入到submap中
    std::vector<std::shared_ptr<const Submap>> insertion_submaps =
        active_submaps_.InsertRangeData(edge_points_in_local,
                                        surf_points_in_local,
                                        pose_estimate);

    //LOG(INFO) << "num of submap is: " << insertion_submaps.front()->num_range_data();
    std::unique_ptr<mapping::InsertionResult> insertion_result =
        absl::make_unique<mapping::InsertionResult>(mapping::InsertionResult{
            std::make_shared<const mapping::TrajectoryNode::Data>(
                mapping::TrajectoryNode::Data{
                    pose_estimate,
                    mapping::RangeData{
                        downsampledEdgeCloud,
                        downsampledSurfCloud,
                        downsampledFullCloud}}),
                    std::move(insertion_submaps)});

    mapping::MatchingResult matching_result{pose_estimate,
                                            mapping::RangeData{edge_points_in_local,
                                                               surf_points_in_local},
                                            std::move(insertion_result)};
    pose_graph_->AddNode(matching_result.insertion_result->constant_data,
                                  matching_result.insertion_result->insertion_submaps);
}

void LocalTrajectoryBuilder::downSamplingToMap(
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_pc_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& full_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);
    downSizeFilterFull.setInputCloud(full_pc_in);
    downSizeFilterFull.filter(*full_pc_out);
}

std::shared_ptr<const Submap> LocalTrajectoryBuilder::getFirstSubmap(){
    return active_submaps_.submaps().front();
}

bool LocalTrajectoryBuilder::last_map(lc_loam::last_mapRequest &req,
		            lc_loam::last_mapResponse &res){
    lastMap();
    return true;
}

bool LocalTrajectoryBuilder::lastOptimization(lc_loam::last_optimization::Request& req,
                                            lc_loam::last_optimization::Response &res){
    res.success = true;
    optimization_flag = true;
    return res.success;
}

bool LocalTrajectoryBuilder::savePCD(lc_loam::save_pcdRequest &req,
		             lc_loam::save_pcdResponse &res){
    std::string saveMapDirectory;
    if(req.destination.empty()){
        saveMapDirectory = savePcdDirectory;
    }
    else{
        saveMapDirectory = std::getenv("HOME") + req.destination;
    }
    // int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    // unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
    save_pcd(saveMapDirectory);
    return true;
}

bool LocalTrajectoryBuilder::save_trajectory(lc_loam::save_trajectoryRequest &req,
		                     lc_loam::save_trajectoryResponse &res){
    std::string trajectoryDir;
    trajectoryDir = std::getenv("HOME") + req.destination;

    std::ofstream fout(trajectoryDir, std::ios::app);
    std::shared_ptr<MapById<NodeId, TrajectoryNode>> trajectory_nodes =
    std::make_shared<MapById<NodeId, TrajectoryNode>>(
            pose_graph_->GetTrajectoryNodes());

    constexpr int trajectory_id = 0;
    auto node_it = trajectory_nodes->BeginOfTrajectory(trajectory_id);
    auto end_it = trajectory_nodes->EndOfTrajectory(trajectory_id);

    nav_msgs::Path path;
    for (; node_it != end_it; ++node_it)
    {
        auto &trajectory_node = trajectory_nodes->at(node_it->id);
        auto &global_pose = trajectory_node.global_pose;
        Eigen::Vector3d t_ = global_pose.translation();
        Eigen::Quaterniond q_ = global_pose.rotation();
        Eigen::Matrix3d rotation = q_.toRotationMatrix();

        // kitti05
        fout << rotation(0, 0) << " " << rotation(0, 1) << " "
             << rotation(0, 2) << " " << t_(0) << " "
             << rotation(1, 0) << " " << rotation(1, 1) << " "
             << rotation(1, 2) << " " << t_(1) << " "
             << rotation(2, 0) << " " << rotation(2, 1) << " "
             << rotation(2, 2) << " " << t_(2);
        fout << std::endl;
    }
    // fout.close();
    std::cout << "gfasyiuft" << std::endl;
    return true;
}

void LocalTrajectoryBuilder::save_pcd(std::string destination){
    std::cout << "-----------save pcd-------------" << std::endl;
    std::cout << "save destination: " << destination << std::endl;
    pcl::io::savePCDFileBinary(destination, *point_cloud_map_);
}

void LocalTrajectoryBuilder::publishRosTf(const ros::Time& pointcloud_time){

     Eigen::Quaterniond q_current(odom.rotation());

     static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(odom.translation().x(),
                                    odom.translation().y(), odom.translation().z()) );
     tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
     transform.setRotation(q);
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

     nav_msgs::Odometry laserOdometry;
     laserOdometry.header.frame_id = "map";
     laserOdometry.child_frame_id = "base_link";
     laserOdometry.header.stamp = pointcloud_time;
     laserOdometry.pose.pose.orientation.x = q_current.x();
     laserOdometry.pose.pose.orientation.y = q_current.y();
     laserOdometry.pose.pose.orientation.z = q_current.z();
     laserOdometry.pose.pose.orientation.w = q_current.w();
     laserOdometry.pose.pose.position.x = odom.translation().x();
     laserOdometry.pose.pose.position.y = odom.translation().y();
     laserOdometry.pose.pose.position.z = odom.translation().z();
     local_path_publisher.publish(laserOdometry);
}


visualization_msgs::MarkerArray LocalTrajectoryBuilder::GetConstraintList(){
    visualization_msgs::MarkerArray constraint_list;
    int marker_id = 0;

    // 1 内部子图约束, 非全局约束, rviz中显示的最多的约束
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  // note: Marker::LINE_LIST: 每对点之间画一条线, eg: 0-1, 2-3, 4-5
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = "map";
  constraint_intra_marker.scale.x = 0.025;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  // 将该标记和其他数量较少的标记设置z为略高于帧内约束标记, 以确保它们可见.
  residual_intra_marker.pose.position.z = 0.1;

  // 外部子图约束, 回环约束, 全局约束
  visualization_msgs::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  // 4 Inter residuals, same trajectory
  visualization_msgs::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses =
      pose_graph_->GetTrajectoryNodePoses();
  const auto submap_poses = pose_graph_->GetAllSubmapPoses();
  const auto constraints = pose_graph_->constraints();

  // 将约束信息填充到6种marker里
  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;

    // 根据不同情况,将constraint_marker与residual_marker 指到到不同的maker类型上

    // 子图内部的constraint,对应第一种与第二种marker
    if (constraint.tag ==
        mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;

      // 各种轨迹的子图的颜色映射-添加轨迹ID以确保不同的起始颜色 还要添加25的固定偏移量, 以避免与轨迹具有相同的颜色.
      color_constraint = ToMessage(
                 GetColor(constraint.submap_id.submap_index +
                                      constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    }
    else {
      // 相同轨迹内,子图外部约束, 对应第三种与第四种marker
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow 亮黄色
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      }
      // Bright cyan 亮青色
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    // 设置颜色信息
    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    // 在submap_poses中找到约束对应的submap_id
    const auto submap_it = submap_poses.find(constraint.submap_id);
    // 没找到就先跳过
    if (submap_it == submap_poses.end()) {
      continue;
    }
    // 子图的坐标
    const auto& submap_pose = submap_it->data.pose;

    // 在trajectory_node_poses中找到约束对应的node_id
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    // 节点在global坐标系下的坐标
    const auto& trajectory_node_pose = node_it->data.global_pose;
    // 根据子图坐标与约束的坐标变换算出约束的另一头的坐标
    const transform::Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    // 将子图与节点间的约束放进不同类型的marker中
    constraint_marker->points.push_back(
        transform::ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        transform::ToGeometryMsgPoint(constraint_pose.translation()));

    // 两种方式计算出的节点坐标不会完全相同, 将这个差值作为残差发布出来
    residual_marker->points.push_back(
        transform::ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        transform::ToGeometryMsgPoint(trajectory_node_pose.translation()));
  } // end for

  // 将填充完数据的Marker放到MarkerArray中
  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  return constraint_list;
}


void LocalTrajectoryBuilder::publishPointCloud(
               const pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_publish,
               const ros::Time& pointcloud_time,
               const ros::Publisher& pub){
    sensor_msgs::PointCloud2 publish_map;
    pcl::toROSMsg(*map_to_publish, publish_map);
    publish_map.header.frame_id = "map";
    publish_map.header.stamp = pointcloud_time;
    pub.publish(publish_map);
}

void LocalTrajectoryBuilder::publishMap(
				 const ros::Time &pointcloud_time,
				 const ros::Publisher &pub){
      if(getFirstSubmap()->insertion_finished()){
                // LOG(INFO) << "-----------full map!----------";
                *fullMap += *(getFirstSubmap()->conerMap());
                *fullMap += *(getFirstSubmap()->surfMap());
                // *fullMap += *(getFirstSubmap()->fullMap());

                downSizeFullMap.setInputCloud(fullMap);
                downSizeFullMap.filter(*fullMap);
                //      ++num_of_submaps;
      }
      publishPointCloud(fullMap, pointcloud_time, pubMap);
}

void LocalTrajectoryBuilder::PublishConstraintList(
			const ::ros::WallTimerEvent &unused_timer_event){
    if(constraint_list_publisher_.getNumSubscribers() > 0) {
        absl::MutexLock lock(&mutex_);
        constraint_list_publisher_.publish(GetConstraintList());
    }
 }

 void LocalTrajectoryBuilder::PublishAfterOptimization(
                  const ::ros::WallTimerEvent& unused_timer_event){
    if(global_path_publisher.getNumSubscribers() > 0){
    std::shared_ptr<MapById<NodeId, TrajectoryNode>> trajectory_nodes =
    std::make_shared<MapById<NodeId, TrajectoryNode>>(
            pose_graph_->GetTrajectoryNodes());

    constexpr int trajectory_id = 0;
    auto node_it = trajectory_nodes->BeginOfTrajectory(trajectory_id);
    auto end_it = trajectory_nodes->EndOfTrajectory(trajectory_id);

    nav_msgs::Path path;
    for (; node_it != end_it; ++node_it)
    {
        auto &trajectory_node = trajectory_nodes->at(node_it->id);
        auto &global_pose = trajectory_node.global_pose;
        Eigen::Vector3d t_ = global_pose.translation();
        Eigen::Quaterniond q_ = global_pose.rotation();

        geometry_msgs::PoseStamped globalOdometry;
        globalOdometry.header.frame_id = "map";
        globalOdometry.header.stamp = ros::Time::now();
        globalOdometry.pose.orientation.x = q_.x();
        globalOdometry.pose.orientation.y = q_.y();
        globalOdometry.pose.orientation.z = q_.z();
        globalOdometry.pose.orientation.w = q_.w();
        globalOdometry.pose.position.x = t_.x();
        globalOdometry.pose.position.y = t_.y();
        globalOdometry.pose.position.z = t_.z();
        path.poses.push_back(globalOdometry);
    }
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    global_path_publisher.publish(path);
    }
 }

 void LocalTrajectoryBuilder::lastMap(){

    std::shared_ptr<MapById<NodeId, TrajectoryNode>> trajectory_nodes =
    std::make_shared<MapById<NodeId, TrajectoryNode>>(
            pose_graph_->GetTrajectoryNodes());

    constexpr int trajectory_id = 0;
    size_t trajectory_nodes_size =
                 trajectory_nodes->SizeOfTrajectoryOrZero(trajectory_id);
    if(last_trajectory_nodes_size_ == trajectory_nodes_size)
        return;
    last_trajectory_nodes_size_ = trajectory_nodes_size;
    absl::MutexLock lock(&point_cloud_map_mutex_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr node_point_cloud_edge(
                              new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr node_point_cloud_surf(
                              new pcl::PointCloud<pcl::PointXYZI>());
    auto node_it = trajectory_nodes->BeginOfTrajectory(trajectory_id);
    auto end_it = trajectory_nodes->EndOfTrajectory(trajectory_id);

    long num_ponts = 0;
    for (; node_it != end_it; ++node_it)
    {
        auto &trajectory_node = trajectory_nodes->at(node_it->id);
        auto &point_cloud_edge =
           trajectory_node.constant_data->range_data_raw.edge_data_in_local;
        auto &point_cloud_surf =
           trajectory_node.constant_data->range_data_raw.surf_data_in_local;
        if(trajectory_node.constant_data != nullptr){
             num_ponts += point_cloud_edge->size();
             num_ponts += point_cloud_surf->size();
        }
    }
    node_it = trajectory_nodes->BeginOfTrajectory(trajectory_id);
    point_cloud_map_->reserve(num_ponts);
    for (; node_it != end_it; ++node_it)
    {
        auto &trajectory_node = trajectory_nodes->at(node_it->id);
        auto &point_cloud_edge =
           trajectory_node.constant_data->range_data_raw.edge_data_in_local;
        auto &point_cloud_surf =
           trajectory_node.constant_data->range_data_raw.surf_data_in_local;
        auto &global_pose = trajectory_node.global_pose;

        if(trajectory_node.constant_data != nullptr){
            node_point_cloud_edge->clear();
            node_point_cloud_surf->clear();
            node_point_cloud_edge->resize(point_cloud_edge->size());
            node_point_cloud_surf->resize(point_cloud_surf->size());

            transform::tranformRageData(point_cloud_edge, node_point_cloud_edge, global_pose);
            transform::tranformRageData(point_cloud_edge, node_point_cloud_edge, global_pose);
            *point_cloud_map_ += *node_point_cloud_edge;
            *point_cloud_map_ += *node_point_cloud_surf;
        }
        downSizeFullMap.setInputCloud(point_cloud_map_);
        downSizeFullMap.filter(*point_cloud_map_);

        ros_point_cloud_map_.data.clear();
        pcl::toROSMsg(*point_cloud_map_, ros_point_cloud_map_);
        ros_point_cloud_map_.header.stamp = ros::Time::now();
        ros_point_cloud_map_.header.frame_id = "map";
        pubLastMap.publish(ros_point_cloud_map_);
    }
}
