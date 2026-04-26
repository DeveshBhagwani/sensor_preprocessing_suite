#include "sensor_preprocessing_suite/sensor_preprocessing_component.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rmw/qos_profiles.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace sensor_preprocessing_suite
{

SensorPreprocessingComponent::SensorPreprocessingComponent(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("sensor_preprocessing_component", node_options)
{
  load_settings();

  imu_in_.subscribe(this, "imu/in", rmw_qos_profile_sensor_data);
  points_in_.subscribe(this, "points/in", rmw_qos_profile_sensor_data);

  imu_out_ = create_publisher<sensor_msgs::msg::Imu>("imu/out", rclcpp::SensorDataQoS());
  points_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points/out",
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
    sensor_msgs::msg::PointCloud2 clean_points = core_.clean_points(*laser_points);
    imu_out_->publish(clean_imu);
    points_out_->publish(clean_points);
  } catch (const std::runtime_error & problem) {
    RCLCPP_WARN(get_logger(), "%s", problem.what());
  }
}

void SensorPreprocessingComponent::load_settings()
{
  const double time_limit = declare_parameter<double>("time_limit", 0.02);
  const double near_limit = declare_parameter<double>("near_limit", 0.1);
  const double far_limit = declare_parameter<double>("far_limit", 200.0);

  core_.set_time_limit(time_limit);
  core_.set_noise_limits(near_limit, far_limit);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_preprocessing_suite::SensorPreprocessingComponent)
