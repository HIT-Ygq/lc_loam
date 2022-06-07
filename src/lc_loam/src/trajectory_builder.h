
#pragma once

//std lib
#include <string>
#include <math.h>
#include <vector>
#include <mutex>
#include <string>
#include <fstream>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "absl/memory/memory.h"

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "pose_graph.h"
#include "graph_node.h"
#include "options.h"
#include "submap/activeSubmap.h"

#include "io/visual_color.h"
#include "msg_conversion.h"
#include "scan_matcher/scan_matcher.h"
#include "scan_matcher/ceres_cost_function.h"
#include "lc_loam/last_optimization.h"
#include "lc_loam/save_pcd.h"
#include "lc_loam/save_trajectory.h"
#include "lc_loam/last_map.h"

using ::mapping::MapById;
using ::mapping::NodeId;
using ::mapping::TrajectoryNode;

constexpr double kConstraintPublishPeriodSec = 0.5;
constexpr char kConstraintListTopic[] = "constraint_list";
constexpr int kLatestOnlyPublisherQueueSize = 1;
class LocalTrajectoryBuilder
{
    public:
		LocalTrajectoryBuilder();
		LocalTrajectoryBuilder(const common::NodeOptions& node_options);

        // 调用主函数
		void odom_estimation();
		// 回调函数
		void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
		void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
		void velodyneFullHandle(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
		bool lastOptimization(lc_loam::last_optimizationRequest &req,
							  lc_loam::last_optimizationResponse &res);
		std::shared_ptr<const Submap> getFirstSubmap();
		void lastMap();

		bool optimization_flag = false;
		// ros节点与消息发布
		ros::NodeHandle nh;
		ros::ServiceServer last_map_;
		ros::ServiceServer last_optimization;
		ros::ServiceServer save_map;
		ros::ServiceServer save_pose;
		std::unique_ptr<mapping::PoseGraph> pose_graph_;

	private:

        // 发布与订阅的消息
		ros::Publisher local_path_publisher;
        ros::Publisher pubSubmap_corner;
        ros::Publisher pubSubmap_surf;
        ros::Publisher pubMap;
        ros::Publisher pubLastMap;
		ros::Publisher global_path_publisher;
		ros::Publisher constraint_list_publisher_;
        std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;
		ros::Subscriber subEdgeLaserCloud;
		ros::Subscriber subSurfLaserCloud;
		ros::Subscriber filterPointCloud;

		std::vector<ros::WallTimer> wall_timers_;
        // 前端匹配位姿
        Eigen::Isometry3d odom;
		Eigen::Isometry3d last_odom;
		// odom_estimation的参数
		bool is_odom_inited = false;
        double total_time = 0;
        int total_frame = 0;
		double scan_period = 0.1;
		//optimization count
		int optimization_count;
		//运动过滤参数
		double speed_filter = 0.1;
		// tag: 子图个数
		int num_of_submaps = 0;
		int num_of_pointcloud = 5;
		// local map
		// 前端维护两个子图， 保存的是相邻两个子图的shared_ptr共享指针
		ActiveSubmaps active_submaps_;
		// ceres匹配器
		optimization::CeresScanMatcher ceres_scan_matcher;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterFull;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFullMap;

        // result
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_map_;
		pcl::PointCloud<pcl::PointXYZI>::Ptr fullMap;
		nav_msgs::Path local_path;
        // 后端
		common::ThreadPool thread_pool_;
		std::mutex mutex_lock;
		absl::Mutex mutex_;

		std::string savePcdDirectory = "/xxx";

		size_t last_trajectory_nodes_size_ = 0;
		absl::Mutex point_cloud_map_mutex_;
		sensor_msgs::PointCloud2 ros_point_cloud_map_;


		void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud,
		                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);

		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in,
		                       pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out,
		                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in,
		                       pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out,
							   const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_pc_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& full_pc_out);
		void addRangeData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
		                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in,
							  const pcl::PointCloud<pcl::PointXYZI>::Ptr& full_in);

		void publishRosTf(const ros::Time& pointcloud_time);
		void publishPointCloud(
			const pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_publish,
			const ros::Time& pointcloud_time,
			const ros::Publisher& pub);
		void publishMap(const ros::Time &pointcloud_time,
						const ros::Publisher &pub);
		void PublishConstraintList(
			const ::ros::WallTimerEvent &unused_timer_event);
		void PublishAfterOptimization(
			const ::ros::WallTimerEvent& unused_timer_event);

		void save_pcd(std::string destination);
        bool savePCD(lc_loam::save_pcdRequest &req,
		             lc_loam::save_pcdResponse &res);
        bool save_trajectory(lc_loam::save_trajectoryRequest &req,
		                     lc_loam::save_trajectoryResponse &res);
		bool last_map(lc_loam::last_mapRequest &req,
		            lc_loam::last_mapResponse &res);
		visualization_msgs::MarkerArray GetConstraintList();
		//::std_msgs::ColorRGBA ToMessage(const std::array<float, 3>& color);
		int counter = 0;

};
