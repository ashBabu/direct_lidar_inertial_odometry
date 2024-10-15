#include "lidar_mapping/lm.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "lidar_mapping/srv/save_pcd.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

class lidar_mapping::MapNode: public rclcpp::Node {

public:

  MapNode();
  ~MapNode();

  void start();

private:

  void getParams();

  void callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

  void savePCD(std::shared_ptr<lidar_mapping::srv::SavePCD::Request> req,
               std::shared_ptr<lidar_mapping::srv::SavePCD::Response> res);


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::CallbackGroup::SharedPtr keyframe_cb_group, save_pcd_cb_group;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

  rclcpp::Service<lidar_mapping::srv::SavePCD>::SharedPtr save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr lidar_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  double leaf_size_;

};
