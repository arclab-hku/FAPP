#ifndef _mot_mapping_H_
#define _mot_mapping_H_

#include <chrono>
#include <queue>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <obj_state_msgs/State.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
// Algorithm
#include "DBSCAN_kdtree.h"
#include "Hungarian.h"
#include "target_ekf.hpp"
#include "ikd_Tree.h"
#include "ikd_Tree_impl.h"

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Points;
typedef pcl::PointCloud<PointType>::Ptr PointsPtr;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

struct MapParam {
  // DBSCAN Params
  int core_pts;
  double tolerance;
  int min_cluster;
  int max_cluster;
  // Dynamic Detection Params
  double thresh_dist;
  double thresh_var;
  // Lidar Params
  Eigen::Vector3d Range;
  // ROS Params
  std::string odom_topic;
  std::string lidar_topic;
};

struct MapData {
  Eigen::Matrix3d R; //Currnet IMU Orientation
  Eigen::Vector3d T; //Currnet IMU Positon
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ObjectState {
  int id; 
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d size;
};

namespace mot_mapping {

class MappingRos {

private:
  pcl::VoxelGrid<PointType> vox;
  DBSCANKdtreeCluster<PointType> dbscan;
  KD_TREE<PointType>::Ptr ikdtree_ptr;

  PointsPtr Remaining_Points;
  int frame_num;
  std::deque<PointsPtr> previous_points;
  std::deque<PointsPtr> buffer;
  std::deque<PointsPtr> input_point;
  std::vector<Eigen::Vector3d> previous_p;
  std::vector<Eigen::Vector3d> previous_v;
  std::vector<pcl::PointIndices> deleted_indices;
  std::vector<BoxPointType> delete_boxes;

  std::vector<std::shared_ptr<Ekf>> trackers;
  std::vector<ObjectState> detections;

  ros::Time t_start;

private:
  ros::NodeHandle &nh;
  ros::Timer ekf_predict_timer_, map_pub_timer_;
  ros::Publisher cloudPub, mapPub, staticMapPub, edgePub;
  ros::Publisher objectPosePub, statesPub;
  int id;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  typedef boost::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  SynchronizerCloudOdom sync_cloud_odom_;

  unique_ptr<MapData> md_;
  unique_ptr<MapParam> mp_;

  float calc_dist(PointType p1, PointType p2);
  void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths);
  void pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w);
  void pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T);

  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& odom);
  void ekfPredictCallback(const ros::TimerEvent& e);
  void mapPubCallback(const ros::TimerEvent& e);
  void visualizeFunction(const std::vector<std::pair<int, int>> pairs);

  double iou(ObjectState state1, ObjectState state2);

public:

  MappingRos(ros::NodeHandle &nh);
  ~MappingRos();
  
  void init();

};

}

#endif