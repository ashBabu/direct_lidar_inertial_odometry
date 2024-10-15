#include "lidar_mapping/map.h"
#include "lidar_mapping/utils.h"

lidar_mapping::MapNode::MapNode(): Node("lidar_map_node") {

  this->getParams();

  this->keyframe_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto keyframe_sub_opt = rclcpp::SubscriptionOptions();
  keyframe_sub_opt.callback_group = this->keyframe_cb_group;
  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframes", 10,
      std::bind(&lidar_mapping::MapNode::callbackKeyframe, this, std::placeholders::_1), keyframe_sub_opt);

  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);

  this->save_pcd_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->save_pcd_srv = this->create_service<lidar_mapping::srv::SavePCD>("save_pcd",
      std::bind(&lidar_mapping::MapNode::savePCD, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->save_pcd_cb_group);

  this->lidar_map = std::make_shared<pcl::PointCloud<PointType>>();

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

lidar_mapping::MapNode::~MapNode() {}

void lidar_mapping::MapNode::getParams() {

  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<double>("map/sparse/leafSize", 0.5);

  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("map/sparse/leafSize", this->leaf_size_);
}

void lidar_mapping::MapNode::start() {
}

void lidar_mapping::MapNode::callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->lidar_map += *keyframe_pcl;

  // publish full map
  if (this->lidar_map->points.size() == this->lidar_map->width * this->lidar_map->height) {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->lidar_map, map_ros);
    map_ros.header.stamp = this->now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub->publish(map_ros);
  } 
}

void lidar_mapping::MapNode::savePCD(std::shared_ptr<lidar_mapping::srv::SavePCD::Request> req,
                            std::shared_ptr<lidar_mapping::srv::SavePCD::Response> res) {

  pcl::PointCloud<PointType>::Ptr m = std::make_shared<pcl::PointCloud<PointType>>(*this->lidar_map);

  float leaf_size = req->leaf_size;
  std::string p = req->save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/lidar_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save timestamped map
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  
  int ret = pcl::io::savePCDFileBinary(p + "/lidar_map_" +  std::to_string(timestamp)+ ".pcd", *m);
  res->success = ret == 0;

  if (res->success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }
}
