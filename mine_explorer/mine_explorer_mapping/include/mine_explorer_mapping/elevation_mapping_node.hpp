#ifndef MINE_EXPLORER_MAPPING__ELEVATION_MAPPING_NODE_HPP_
#define MINE_EXPLORER_MAPPING__ELEVATION_MAPPING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>

class ElevationMappingNode : public rclcpp::Node
{
public:
  explicit ElevationMappingNode();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool transformPointCloud(
      const sensor_msgs::msg::PointCloud2 &input_cloud,
      sensor_msgs::msg::PointCloud2 &output_cloud,
      const std::string &target_frame,
      const std::string &source_frame);

  bool getFramePosition(const std::string &target_frame,
                        const std::string &source_frame,
                        double &x, double &y);
                      
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsInBoundingBox(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelizePointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

  void updateElevationMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map);
  bool getRollingWindowSubmap(grid_map::GridMap &submap);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  float x_min_, x_max_;
  float y_min_, y_max_;
  float z_min_, z_max_;

  float voxel_size_;

  grid_map::GridMap elevation_map_;
  float map_resolution_;
  float map_length_x_, map_length_y_;
  float sub_map_length_x_, sub_map_length_y_;
};

#endif