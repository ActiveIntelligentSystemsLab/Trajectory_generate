#ifndef TRAVERSABILITY_MAP_GLOBAL_H
#define TRAVERSABILITY_MAP_GLOBAL_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_geometry/pinhole_camera_model.h>


// GridMap
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// ROS messages
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include "rtabmap_create_data/CreateData.h"

// CV Bridge
#include <cv_bridge/cv_bridge.h>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// TF2
#include <tf2_ros/transform_listener.h>

// General
#include <cmath>

// Octomap
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>

// RTAB-Map
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/GetMap.h>


// image_geometry
#include <image_geometry/pinhole_camera_model.h>

// depth_image_proc
// #include <depth_image_proc/depth_traits.h>

// OpenCV
#include <opencv2/opencv.hpp>

// RTAB-Map
#include <rtabmap/core/Compression.h>

// TF2
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define PRED_PROB_MAX 0.7
#define OBSV_PROB_MAX 0.7

class MapToImage {
private :
  // Service server
  ros::ServiceServer server_create_voxel_map_, server_create_image_data_;
  ros::ServiceClient client_get_map_data_;
  ros::Publisher vis_pub;
  rtabmap_ros::MapData mapData_;
  ros::Publisher data_publish;
  // ros::Timer timer_:
  // transform method
  // tf2_ros::TransformBroadcaster dynamic_br_;
  float cx, cy, fx, fy;
  float width, height;

public :
  // Constructor
  MapToImage(ros::NodeHandle nh_);
  // Destructor
  ~MapToImage();
private :
  // Callback for PointCloud subscriber
  bool callbackGetDataset(rtabmap_create_data::CreateData::Request & req, rtabmap_create_data::CreateData::Response & res);
  bool callbackGetImageData(rtabmap_create_data::CreateData::Request & req, rtabmap_create_data::CreateData::Response & res);
  //tf2::Transform changePointTF(double x, double y, double z, double qx, double qy, double qz, double qw);
  tf2::Transform subPointTF(tf2::Transform tf2_original, tf2::Transform tf2_target);
  tf2::Transform changePointTF(geometry_msgs::Pose pose);
  tf2::Transform changeTransformTF(geometry_msgs::Transform trans);

  // transform method
  tf2_ros::TransformBroadcaster dynamic_br_;
  std::vector<cv::Point> crossingCheck(std::vector<cv::Point> point_list, float image_x, float image_y, float image_widht, float image_height);
  std::vector<double> leastSquares(std::vector<cv::Point> point_list);
  void generateMarker(visualization_msgs::Marker& publish_point, int color_code);
  void fitLineRansac(const std::vector<cv::Point2f>& points, cv::Vec4f &line, int iterations, double sigma, double k_min, double k_max);
  //void fitLine3dRansac(const std::vector<cv::Point2f>& points, cv::Vec4f &line, int iterations, double sigma);
  void recordTransForm(geometry_msgs::Transform trans);
  void settingPosition(geometry_msgs::Pose trans);
  void temprecordTransForm(geometry_msgs::Pose trans, int index_number);
};

#endif // TRAVERSABILITY_GLOBAL_MAP_H
