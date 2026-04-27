#ifndef SENSOR_PREPROCESSING_SUITE__SENSOR_PREPROCESSING_COMPONENT_HPP_
#define SENSOR_PREPROCESSING_SUITE__SENSOR_PREPROCESSING_COMPONENT_HPP_

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_preprocessing_suite/sensor_preprocessing_core.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_preprocessing_suite
{

class SensorPreprocessingComponent : public rclcpp::Node
{
public:
  explicit SensorPreprocessingComponent(const rclcpp::NodeOptions & node_options);

private:
  void synced_data(
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu_data,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & laser_points);

  void load_settings();
  bool read_frame_link(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp_time,
    geometry_msgs::msg::Transform & frame_link);

  SensorPreprocessingCore core_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_out_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_out_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_out_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_out_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr flat_out_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_in_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_in_;
  using sync_rule = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu,
    sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<sync_rule>> sync_;
  tf2_ros::Buffer frame_buffer_;
  tf2_ros::TransformListener frame_reader_;
  std::string imu_frame_;
  std::string lidar_frame_;
};

}

#endif
