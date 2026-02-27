#include "mine_explorer_mapping/elevation_mapping_node.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>

ElevationMappingNode::ElevationMappingNode() : Node("elevation_mapping_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensors/lidar/data",
        10,
        std::bind(&ElevationMappingNode::pointCloudCallback, this, std::placeholders::_1));

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensors/lidar/data_filtered",
        10);

    pub_voxelized_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensors/lidar/data_voxelized",
        10);

    pub_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/mapping/elevation_map",
        10);

    x_min_ = -0.7;
    x_max_ = 0.7;
    y_min_ = -0.45;
    y_max_ = 0.45;
    z_min_ = -0.25;
    z_max_ = 0.8;

    voxel_size_ = 0.05f;

    map_resolution_ = 0.05f;
    map_length_x_ = 100.0f;
    map_length_y_ = 100.0f;
        
    elevation_map_.setFrameId("map");
    elevation_map_.setGeometry(grid_map::Length(map_length_x_, map_length_y_), map_resolution_);
    elevation_map_.setBasicLayers({"elevation"});
    elevation_map_.add("elevation", 0.0f);

    RCLCPP_INFO(this->get_logger(), "Elevation mapping node started.");
}

void ElevationMappingNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Transform lidar_base_link -> base_link
    sensor_msgs::msg::PointCloud2 cloud_transformed;

    try
    {
        geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform(
            "base_link",
            msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));

        tf2::doTransform(*msg, cloud_transformed, transform);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
        return;
    }

    // Bounding Box Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_transformed, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &point : pcl_cloud->points)
    {
        bool inside_robot =
        (point.x > x_min_ && point.x < x_max_) &&
        (point.y > y_min_ && point.y < y_max_) &&
        (point.z > z_min_ && point.z < z_max_);

        if (!inside_robot)
        {
        filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);

    output.header.frame_id = "base_link";
    output.header.stamp = msg->header.stamp;

    pub_cloud_->publish(output);

    // Voxel Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*voxel_cloud);

    sensor_msgs::msg::PointCloud2 voxel_msg;
    pcl::toROSMsg(*voxel_cloud, voxel_msg);

    voxel_msg.header.frame_id = "base_link";
    voxel_msg.header.stamp = msg->header.stamp;

    pub_voxelized_cloud_->publish(voxel_msg);

    // Transform base_link->map
    sensor_msgs::msg::PointCloud2 voxel_map;
    try
    {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_.lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));

        tf2::doTransform(voxel_msg, voxel_map, transform);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
        return;
    }

    // Elevation map
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(voxel_map, *voxel_cloud_map);

    for (const auto &point : voxel_cloud_map->points) {
        float x = static_cast<float>(point.x);
        float y = static_cast<float>(point.y);
        float z = static_cast<float>(point.z);

        grid_map::Position position(x, y);

        grid_map::Index index;
        if (elevation_map_.getIndex(grid_map::Position(x, y), index)) {
            float &cell = elevation_map_.at("elevation", index);
            if (z > cell) {
                cell = z;
            }
        }
    }

    // Rolling Window
    double robot_x = 0.0;
    double robot_y = 0.0;
    try
    {
        geometry_msgs::msg::TransformStamped tf_robot =
            tf_buffer_.lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));

        robot_x = tf_robot.transform.translation.x;
        robot_y = tf_robot.transform.translation.y;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Rolling map TF failed: %s", ex.what());
        return;
    }

    float window_size = 10.0f;
    grid_map::Position robot_position(robot_x, robot_y);

    bool success = false;

    grid_map::GridMap submap = elevation_map_.getSubmap(
        robot_position,
        grid_map::Length(window_size, window_size),
        success);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to get submap around robot!");
        return;
    }

    auto map_msg_ptr = grid_map::GridMapRosConverter::toMessage(submap);
    pub_map_->publish(*map_msg_ptr);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMappingNode>());
  rclcpp::shutdown();
  return 0;
}