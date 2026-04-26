#include "sensor_preprocessing_suite/sensor_preprocessing_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "sensor_msgs/msg/point_field.hpp"

namespace sensor_preprocessing_suite
{

SensorPreprocessingCore::SensorPreprocessingCore()
: time_limit_(0.02),
  near_limit_(0.1),
  far_limit_(200.0)
{
}

void SensorPreprocessingCore::set_time_limit(double time_limit)
{
  time_limit_ = time_limit;
}

void SensorPreprocessingCore::set_noise_limits(double near_limit, double far_limit)
{
  near_limit_ = near_limit;
  far_limit_ = far_limit;
}

bool SensorPreprocessingCore::times_match(
  const sensor_msgs::msg::Imu & imu_data,
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  const double imu_time = read_time(imu_data.header.stamp);
  const double laser_time = read_time(laser_points.header.stamp);
  const double time_difference = std::fabs(imu_time - laser_time);
  return time_difference <= time_limit_;
}

sensor_msgs::msg::Imu SensorPreprocessingCore::remove_gravity(
  const sensor_msgs::msg::Imu & imu_data) const
{
  sensor_msgs::msg::Imu clean_imu = imu_data;
  const std::vector<double> gravity_pull = gravity_pull_in_body(imu_data);
  clean_imu.linear_acceleration.x = imu_data.linear_acceleration.x - gravity_pull[0];
  clean_imu.linear_acceleration.y = imu_data.linear_acceleration.y - gravity_pull[1];
  clean_imu.linear_acceleration.z = imu_data.linear_acceleration.z - gravity_pull[2];
  return clean_imu;
}

sensor_msgs::msg::PointCloud2 SensorPreprocessingCore::clean_points(
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  const point_place point_place_data = find_point_place(laser_points);
  if (!point_place_data.ready) {
    throw std::runtime_error("point cloud is missing x y z fields");
  }

  sensor_msgs::msg::PointCloud2 clean_points = laser_points;
  clean_points.data.clear();
  clean_points.data.reserve(laser_points.data.size());

  const std::size_t point_count =
    static_cast<std::size_t>(laser_points.width) * static_cast<std::size_t>(laser_points.height);

  std::size_t kept_points = 0;
  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    const simple_point one_point = read_point(laser_points, point_number, point_place_data);
    if (!point_is_clean(one_point)) {
      continue;
    }

    write_point(clean_points, laser_points, point_number);
    ++kept_points;
  }

  clean_points.width = static_cast<std::uint32_t>(kept_points);
  clean_points.height = 1u;
  clean_points.row_step = clean_points.point_step * clean_points.width;
  clean_points.is_dense = true;
  return clean_points;
}

SensorPreprocessingCore::point_place SensorPreprocessingCore::find_point_place(
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  point_place point_place_data{};
  point_place_data.x_place = 0;
  point_place_data.y_place = 0;
  point_place_data.z_place = 0;
  point_place_data.ready = false;

  bool x_found = false;
  bool y_found = false;
  bool z_found = false;

  for (const auto & field_data : laser_points.fields) {
    if (field_data.name == "x") {
      point_place_data.x_place = field_data.offset;
      x_found = true;
    } else if (field_data.name == "y") {
      point_place_data.y_place = field_data.offset;
      y_found = true;
    } else if (field_data.name == "z") {
      point_place_data.z_place = field_data.offset;
      z_found = true;
    }
  }

  point_place_data.ready = x_found && y_found && z_found;
  return point_place_data;
}

SensorPreprocessingCore::simple_point SensorPreprocessingCore::read_point(
  const sensor_msgs::msg::PointCloud2 & laser_points,
  std::size_t point_number,
  const point_place & point_place_data) const
{
  simple_point one_point{};
  const std::size_t start_place = point_number * laser_points.point_step;

  std::memcpy(
    &one_point.x,
    &laser_points.data[start_place + point_place_data.x_place],
    sizeof(float));
  std::memcpy(
    &one_point.y,
    &laser_points.data[start_place + point_place_data.y_place],
    sizeof(float));
  std::memcpy(
    &one_point.z,
    &laser_points.data[start_place + point_place_data.z_place],
    sizeof(float));

  return one_point;
}

void SensorPreprocessingCore::write_point(
  sensor_msgs::msg::PointCloud2 & clean_points,
  const sensor_msgs::msg::PointCloud2 & laser_points,
  std::size_t point_number) const
{
  const std::size_t start_place = point_number * laser_points.point_step;
  const auto start_data = laser_points.data.begin() + static_cast<std::ptrdiff_t>(start_place);
  const auto end_data = start_data + static_cast<std::ptrdiff_t>(laser_points.point_step);
  clean_points.data.insert(clean_points.data.end(), start_data, end_data);
}

double SensorPreprocessingCore::read_time(
  const builtin_interfaces::msg::Time & stamp_time) const
{
  return static_cast<double>(stamp_time.sec) +
    static_cast<double>(stamp_time.nanosec) * 1e-9;
}

std::vector<double> SensorPreprocessingCore::gravity_pull_in_body(
  const sensor_msgs::msg::Imu & imu_data) const
{
  const double x = imu_data.orientation.x;
  const double y = imu_data.orientation.y;
  const double z = imu_data.orientation.z;
  const double w = imu_data.orientation.w;

  const double body_x = 2.0 * 9.81 * (x * z - w * y);
  const double body_y = 2.0 * 9.81 * (y * z + w * x);
  const double body_z = 9.81 * (1.0 - 2.0 * (x * x + y * y));

  return {body_x, body_y, body_z};
}

bool SensorPreprocessingCore::point_is_clean(const simple_point & one_point) const
{
  if (!std::isfinite(one_point.x) || !std::isfinite(one_point.y) || !std::isfinite(one_point.z)) {
    return false;
  }

  const double point_length = std::sqrt(
    static_cast<double>(one_point.x) * static_cast<double>(one_point.x) +
    static_cast<double>(one_point.y) * static_cast<double>(one_point.y) +
    static_cast<double>(one_point.z) * static_cast<double>(one_point.z));

  return point_length >= near_limit_ && point_length <= far_limit_;
}

}
