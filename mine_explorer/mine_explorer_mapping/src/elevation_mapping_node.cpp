#include "mine_explorer_mapping/elevation_mapping_node.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>

ElevationMappingNode::ElevationMappingNode() : Node("elevation_mapping_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    // Topic parameters
    std::string input_cloud_topic =
        this->declare_parameter("input_cloud_topic", "/sensors/lidar/data");
    std::string output_map_topic =
        this->declare_parameter("output_map_topic", "/mapping/elevation_map");

    // Map / voxel parameters
    voxel_size_ = this->declare_parameter("voxel_size", 0.05);
    map_resolution_ = this->declare_parameter("map_resolution", 0.05);

    // Submap parameters
    double sub_map_length = this->declare_parameter("sub_map_length", 10.0);
    sub_map_length_x_ = this->declare_parameter("sub_map_length_x", sub_map_length);
    sub_map_length_y_ = this->declare_parameter("sub_map_length_y", sub_map_length);

    // Map size totale
    map_length_x_ = this->declare_parameter("map_length_x", 100.0);
    map_length_y_ = this->declare_parameter("map_length_y", 100.0);

    // Bounding box parameters
    x_min_ = this->declare_parameter("robot_filter.x_min", -0.7);
    x_max_ = this->declare_parameter("robot_filter.x_max", 0.7);
    y_min_ = this->declare_parameter("robot_filter.y_min", -0.45);
    y_max_ = this->declare_parameter("robot_filter.y_max", 0.45);
    z_min_ = this->declare_parameter("robot_filter.z_min", -0.25);
    z_max_ = this->declare_parameter("robot_filter.z_max", 0.8);

    elevation_map_.setFrameId("map");
    elevation_map_.setGeometry(grid_map::Length(map_length_x_, map_length_y_), map_resolution_);
    elevation_map_.setBasicLayers({"elevation"});
    elevation_map_.add("elevation", 0.0f);

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic,
        10,
        std::bind(&ElevationMappingNode::pointCloudCallback, this, std::placeholders::_1));

    pub_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        output_map_topic,
        10);

    RCLCPP_INFO(this->get_logger(), "Elevation mapping node started.");
    RCLCPP_INFO(this->get_logger(), "\n=========== Parameters ===========");
    RCLCPP_INFO(this->get_logger(), "\n[Topics]");
    RCLCPP_INFO(this->get_logger(), "  input_cloud_topic  : %s", input_cloud_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  output_map_topic   : %s", output_map_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "\n[Mapping]");
    RCLCPP_INFO(this->get_logger(), "  voxel_size         : %.3f", voxel_size_);
    RCLCPP_INFO(this->get_logger(), "  map_resolution     : %.3f", map_resolution_);
    RCLCPP_INFO(this->get_logger(), "  map_size           : %.1f x %.1f", map_length_x_, map_length_y_);
    RCLCPP_INFO(this->get_logger(), "\n[Submap]");
    RCLCPP_INFO(this->get_logger(), "  submap_size        : %.1f x %.1f", sub_map_length_x_, sub_map_length_y_);
    RCLCPP_INFO(this->get_logger(), "\n[Robot filter]");
    RCLCPP_INFO(this->get_logger(), "  x range            : [%.2f : %.2f]", x_min_, x_max_);
    RCLCPP_INFO(this->get_logger(), "  y range            : [%.2f : %.2f]", y_min_, y_max_);
    RCLCPP_INFO(this->get_logger(), "  z range            : [%.2f : %.2f]", z_min_, z_max_);
    RCLCPP_INFO(this->get_logger(), "\n==================================");
}

bool ElevationMappingNode::transformPointCloud(
    const sensor_msgs::msg::PointCloud2 &input_cloud,
    sensor_msgs::msg::PointCloud2 &output_cloud,
    const std::string &target_frame,
    const std::string &source_frame)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));

        tf2::doTransform(input_cloud, output_cloud, transform);
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF failed from %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

bool ElevationMappingNode::getFramePosition(const std::string &target_frame,
                                            const std::string &source_frame,
                                            double &x, double &y)
{
    try
    {
        geometry_msgs::msg::TransformStamped tf_stamp =
            tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));

        x = tf_stamp.transform.translation.x;
        y = tf_stamp.transform.translation.y;
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF failed from %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ElevationMappingNode::filterPointsInBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto &point : input_cloud->points)
    {
        bool inside_robot =
            (point.x > x_min_ && point.x < x_max_) &&
            (point.y > y_min_ && point.y < y_max_) &&
            (point.z > z_min_ && point.z < z_max_);

        if (!inside_robot)
            filtered_cloud->points.push_back(point);
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ElevationMappingNode::voxelizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    auto voxel_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*voxel_cloud);
    return voxel_cloud;
}

void ElevationMappingNode::updateElevationMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map)
{
    for (const auto &point : cloud_map->points)
    {
        grid_map::Index index;
        if (elevation_map_.getIndex(grid_map::Position(point.x, point.y), index))
        {
            float &cell = elevation_map_.at("elevation", index);
            if (point.z > cell) cell = point.z;
        }
    }
}

bool ElevationMappingNode::getRollingWindowSubmap(grid_map::GridMap &submap)
{
    double robot_x = 0.0, robot_y = 0.0;
    if (!getFramePosition("map", "base_link", robot_x, robot_y)) {
        return false;
    }
    grid_map::Position robot_position(robot_x, robot_y);

    bool success = false;
    submap = elevation_map_.getSubmap(
        robot_position,
        grid_map::Length(sub_map_length_x_, sub_map_length_y_),
        success);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to get submap around robot!");
        return false;
    }
    return true;
}

void ElevationMappingNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Transform lidar_base_link -> base_link
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    if (!transformPointCloud(*msg, cloud_transformed, "base_link", msg->header.frame_id)) {
        return;
    }

    // Convert to PCL
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(cloud_transformed, *pcl_cloud);

    // Filter points on robot
    auto filtered_cloud = filterPointsInBoundingBox(pcl_cloud);

    // Voxelize
    auto voxel_cloud = voxelizePointCloud(filtered_cloud);

    // Transform base_link->map
    sensor_msgs::msg::PointCloud2 voxel_msg;
    pcl::toROSMsg(*voxel_cloud, voxel_msg);
    voxel_msg.header.frame_id = "base_link";
    voxel_msg.header.stamp = msg->header.stamp;

    sensor_msgs::msg::PointCloud2 voxel_map;
    if (!transformPointCloud(voxel_msg, voxel_map, "map", "base_link")) {
        return;
    }

    // Update elevation map
    auto voxel_cloud_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(voxel_map, *voxel_cloud_map);
    updateElevationMap(voxel_cloud_map);

    // Rolling Window
    grid_map::GridMap submap;
    if (getRollingWindowSubmap(submap)) {
        auto map_msg_ptr = grid_map::GridMapRosConverter::toMessage(submap);
        pub_map_->publish(*map_msg_ptr);
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMappingNode>());
  rclcpp::shutdown();
  return 0;
}