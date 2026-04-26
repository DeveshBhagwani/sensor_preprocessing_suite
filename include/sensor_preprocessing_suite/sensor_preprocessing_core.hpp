#ifndef SENSOR_PREPROCESSING_SUITE__SENSOR_PREPROCESSING_CORE_HPP_
#define SENSOR_PREPROCESSING_SUITE__SENSOR_PREPROCESSING_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sensor_preprocessing_suite
{

class SensorPreprocessingCore
{
public:
  SensorPreprocessingCore();

  void set_time_limit(double time_limit);
  void set_noise_limits(double near_limit, double far_limit);

  bool times_match(
    const sensor_msgs::msg::Imu & imu_data,
    const sensor_msgs::msg::PointCloud2 & laser_points) const;

  sensor_msgs::msg::Imu remove_gravity(
    const sensor_msgs::msg::Imu & imu_data) const;

  sensor_msgs::msg::PointCloud2 clean_points(
    const sensor_msgs::msg::PointCloud2 & laser_points) const;

private:
  struct point_place
  {
    std::size_t x_place;
    std::size_t y_place;
    std::size_t z_place;
    bool ready;
  };

  struct simple_point
  {
    float x;
    float y;
    float z;
  };

  point_place find_point_place(
    const sensor_msgs::msg::PointCloud2 & laser_points) const;

  simple_point read_point(
    const sensor_msgs::msg::PointCloud2 & laser_points,
    std::size_t point_number,
    const point_place & point_place_data) const;

  void write_point(
    sensor_msgs::msg::PointCloud2 & clean_points,
    const sensor_msgs::msg::PointCloud2 & laser_points,
    std::size_t point_number) const;

  double read_time(
    const builtin_interfaces::msg::Time & stamp_time) const;

  std::vector<double> gravity_pull_in_body(
    const sensor_msgs::msg::Imu & imu_data) const;

  bool point_is_clean(const simple_point & one_point) const;

  double time_limit_;
  double near_limit_;
  double far_limit_;
};

}

#endif
