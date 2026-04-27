#include "sensor_preprocessing_suite/sensor_preprocessing_component.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rmw/qos_profiles.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace sensor_preprocessing_suite
{

SensorPreprocessingComponent::SensorPreprocessingComponent(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("sensor_preprocessing_component", node_options),
  frame_buffer_(get_clock()),
  frame_reader_(frame_buffer_)
{
  load_settings();

  imu_in_.subscribe(this, "imu/in", rmw_qos_profile_sensor_data);
  points_in_.subscribe(this, "points/in", rmw_qos_profile_sensor_data);

  imu_out_ = create_publisher<sensor_msgs::msg::Imu>("imu/out", rclcpp::SensorDataQoS());
  points_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points/out",
    rclcpp::SensorDataQoS());
  ground_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points/ground",
    rclcpp::SensorDataQoS());
  obstacle_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points/obstacle",
    rclcpp::SensorDataQoS());

  sync_ = std::make_shared<message_filters::Synchronizer<sync_rule>>(sync_rule(20), imu_in_, points_in_);
  sync_->registerCallback(
    std::bind(
      &SensorPreprocessingComponent::synced_data,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void SensorPreprocessingComponent::synced_data(
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu_data,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & laser_points)
{
  if (!core_.times_match(*imu_data, *laser_points)) {
    return;
  }

  try {
    sensor_msgs::msg::Imu clean_imu = core_.remove_gravity(*imu_data);
    geometry_msgs::msg::Transform frame_link;
    if (!read_frame_link(lidar_frame_, imu_frame_, laser_points->header.stamp, frame_link)) {
      return;
    }

    clean_imu = core_.turn_imu_to_lidar(clean_imu, frame_link);
    sensor_msgs::msg::PointCloud2 clean_points = core_.clean_points(*laser_points);
    clean_points = core_.fix_twisted_points(clean_points, clean_imu);
    clean_points = core_.downsample_points(clean_points);
    const auto cloud_split = core_.split_ground_points(clean_points);
    imu_out_->publish(clean_imu);
    points_out_->publish(clean_points);
    ground_out_->publish(cloud_split.ground_points);
    obstacle_out_->publish(cloud_split.obstacle_points);
  } catch (const std::runtime_error & problem) {
    RCLCPP_WARN(get_logger(), "%s", problem.what());
  }
}

void SensorPreprocessingComponent::load_settings()
{
  const double time_limit = declare_parameter<double>("time_limit", 0.02);
  const double near_limit = declare_parameter<double>("near_limit", 0.1);
  const double far_limit = declare_parameter<double>("far_limit", 200.0);
  const double box_size = declare_parameter<double>("box_size", 0.5);
  const double floor_height = declare_parameter<double>("floor_height", 0.0);
  const double scan_time = declare_parameter<double>("scan_time", 0.1);
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");

  core_.set_time_limit(time_limit);
  core_.set_noise_limits(near_limit, far_limit);
  core_.set_box_size(box_size);
  core_.set_floor_height(floor_height);
  core_.set_scan_time(scan_time);
}

bool SensorPreprocessingComponent::read_frame_link(
  const std::string & target_frame,
  const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp_time,
  geometry_msgs::msg::Transform & frame_link)
{
  try {
    const auto frame_data = frame_buffer_.lookupTransform(
      target_frame,
      source_frame,
      tf2::timeFromSec(
        static_cast<double>(stamp_time.sec) +
        static_cast<double>(stamp_time.nanosec) * 1e-9));
    frame_link = frame_data.transform;
    return true;
  } catch (const tf2::TransformException & problem) {
    RCLCPP_WARN(get_logger(), "%s", problem.what());
    return false;
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_preprocessing_suite::SensorPreprocessingComponent)
