#ifndef SENSOR_PREPROCESSING_SUITE__SENSOR_PREPROCESSING_CORE_HPP_
#define SENSOR_PREPROCESSING_SUITE__SENSOR_PREPROCESSING_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform.hpp"
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
  void set_box_size(double box_size);
  void set_floor_height(double floor_height);
  void set_scan_time(double scan_time);

  struct split_cloud
  {
    sensor_msgs::msg::PointCloud2 ground_points;
    sensor_msgs::msg::PointCloud2 obstacle_points;
  };

  bool times_match(
    const sensor_msgs::msg::Imu & imu_data,
    const sensor_msgs::msg::PointCloud2 & laser_points) const;

  sensor_msgs::msg::Imu remove_gravity(
    const sensor_msgs::msg::Imu & imu_data) const;

  sensor_msgs::msg::PointCloud2 clean_points(
    const sensor_msgs::msg::PointCloud2 & laser_points) const;

  sensor_msgs::msg::Imu turn_imu_to_lidar(
    const sensor_msgs::msg::Imu & imu_data,
    const geometry_msgs::msg::Transform & frame_link) const;

  sensor_msgs::msg::PointCloud2 downsample_points(
    const sensor_msgs::msg::PointCloud2 & laser_points) const;

  sensor_msgs::msg::PointCloud2 fix_twisted_points(
    const sensor_msgs::msg::PointCloud2 & laser_points,
    const sensor_msgs::msg::Imu & imu_data) const;

  split_cloud split_ground_points(
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

  struct box_number
  {
    int x_box;
    int y_box;
    int z_box;

    bool operator<(const box_number & other_box) const
    {
      if (x_box != other_box.x_box) {
        return x_box < other_box.x_box;
      }

      if (y_box != other_box.y_box) {
        return y_box < other_box.y_box;
      }

      return z_box < other_box.z_box;
    }
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

  void write_point_with_new_place(
    sensor_msgs::msg::PointCloud2 & clean_points,
    const sensor_msgs::msg::PointCloud2 & laser_points,
    std::size_t point_number,
    const point_place & point_place_data,
    const simple_point & new_place) const;

  double read_time(
    const builtin_interfaces::msg::Time & stamp_time) const;

  std::vector<double> gravity_pull_in_body(
    const sensor_msgs::msg::Imu & imu_data) const;

  std::vector<double> turn_xyz(
    double x,
    double y,
    double z,
    const geometry_msgs::msg::Transform & frame_link) const;

  box_number find_box_number(const simple_point & one_point) const;

  simple_point find_box_middle(const box_number & one_box) const;

  simple_point turn_point_by_turn_speed(
    const simple_point & one_point,
    const sensor_msgs::msg::Imu & imu_data,
    double point_fraction) const;

  bool point_is_clean(const simple_point & one_point) const;

  double time_limit_;
  double near_limit_;
  double far_limit_;
  double box_size_;
  double floor_height_;
  double scan_time_;
};

}

#endif
