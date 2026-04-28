#include "sensor_preprocessing_suite/sensor_preprocessing_core.hpp"

#include <cfloat>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <stdexcept>

#include "sensor_msgs/msg/point_field.hpp"

namespace sensor_preprocessing_suite
{

SensorPreprocessingCore::SensorPreprocessingCore()
: time_limit_(0.02),
  near_limit_(0.1),
  far_limit_(200.0),
  box_size_(0.5),
  floor_height_(0.0),
  scan_time_(0.1),
  neighbor_size_(5),
  edge_score_limit_(0.05),
  flat_score_limit_(0.005),
  group_gap_(0.5)
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

void SensorPreprocessingCore::set_box_size(double box_size)
{
  box_size_ = box_size;
}

void SensorPreprocessingCore::set_floor_height(double floor_height)
{
  floor_height_ = floor_height;
}

void SensorPreprocessingCore::set_scan_time(double scan_time)
{
  scan_time_ = scan_time;
}

void SensorPreprocessingCore::set_curve_settings(
  std::size_t neighbor_size,
  double edge_score_limit,
  double flat_score_limit)
{
  neighbor_size_ = neighbor_size;
  edge_score_limit_ = edge_score_limit;
  flat_score_limit_ = flat_score_limit;
}

void SensorPreprocessingCore::set_group_gap(double group_gap)
{
  group_gap_ = group_gap;
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

sensor_msgs::msg::Imu SensorPreprocessingCore::turn_imu_to_lidar(
  const sensor_msgs::msg::Imu & imu_data,
  const geometry_msgs::msg::Transform & frame_link) const
{
  sensor_msgs::msg::Imu clean_imu = imu_data;
  const std::vector<double> turned_pull = turn_xyz(
    imu_data.linear_acceleration.x,
    imu_data.linear_acceleration.y,
    imu_data.linear_acceleration.z,
    frame_link);
  const std::vector<double> turned_speed = turn_xyz(
    imu_data.angular_velocity.x,
    imu_data.angular_velocity.y,
    imu_data.angular_velocity.z,
    frame_link);
  clean_imu.linear_acceleration.x = turned_pull[0];
  clean_imu.linear_acceleration.y = turned_pull[1];
  clean_imu.linear_acceleration.z = turned_pull[2];
  clean_imu.angular_velocity.x = turned_speed[0];
  clean_imu.angular_velocity.y = turned_speed[1];
  clean_imu.angular_velocity.z = turned_speed[2];
  return clean_imu;
}

sensor_msgs::msg::PointCloud2 SensorPreprocessingCore::downsample_points(
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  if (box_size_ <= 0.0) {
    throw std::runtime_error("box size must be greater than zero");
  }

  const point_place point_place_data = find_point_place(laser_points);
  if (!point_place_data.ready) {
    throw std::runtime_error("point cloud is missing x y z fields");
  }

  std::map<box_number, std::size_t> point_group;
  const std::size_t point_count =
    static_cast<std::size_t>(laser_points.width) * static_cast<std::size_t>(laser_points.height);

  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    const simple_point one_point = read_point(laser_points, point_number, point_place_data);
    const box_number one_box = find_box_number(one_point);
    if (point_group.find(one_box) == point_group.end()) {
      point_group.emplace(one_box, point_number);
    }
  }

  sensor_msgs::msg::PointCloud2 downsampled_points = laser_points;
  downsampled_points.data.clear();
  downsampled_points.data.reserve(point_group.size() * laser_points.point_step);

  for (const auto & box_and_point : point_group) {
    const simple_point middle_point = find_box_middle(box_and_point.first);
    write_point_with_new_place(
      downsampled_points,
      laser_points,
      box_and_point.second,
      point_place_data,
      middle_point);
  }

  downsampled_points.width = static_cast<std::uint32_t>(point_group.size());
  downsampled_points.height = 1u;
  downsampled_points.row_step = downsampled_points.point_step * downsampled_points.width;
  downsampled_points.is_dense = true;
  return downsampled_points;
}

sensor_msgs::msg::PointCloud2 SensorPreprocessingCore::fix_twisted_points(
  const sensor_msgs::msg::PointCloud2 & laser_points,
  const sensor_msgs::msg::Imu & imu_data) const
{
  if (scan_time_ <= 0.0) {
    throw std::runtime_error("scan time must be greater than zero");
  }

  const point_place point_place_data = find_point_place(laser_points);
  if (!point_place_data.ready) {
    throw std::runtime_error("point cloud is missing x y z fields");
  }

  sensor_msgs::msg::PointCloud2 straight_points = laser_points;
  straight_points.data.clear();
  straight_points.data.reserve(laser_points.data.size());

  const std::size_t point_count =
    static_cast<std::size_t>(laser_points.width) * static_cast<std::size_t>(laser_points.height);

  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    const simple_point one_point = read_point(laser_points, point_number, point_place_data);
    double point_fraction = 0.0;
    if (point_count > 1) {
      point_fraction =
        static_cast<double>(point_number) / static_cast<double>(point_count - 1);
    }

    const simple_point fixed_point = turn_point_by_turn_speed(one_point, imu_data, point_fraction);
    write_point_with_new_place(
      straight_points,
      laser_points,
      point_number,
      point_place_data,
      fixed_point);
  }

  straight_points.width = laser_points.width;
  straight_points.height = laser_points.height;
  straight_points.row_step = straight_points.point_step * straight_points.width;
  straight_points.is_dense = laser_points.is_dense;
  return straight_points;
}

SensorPreprocessingCore::split_cloud SensorPreprocessingCore::split_ground_points(
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  const point_place point_place_data = find_point_place(laser_points);
  if (!point_place_data.ready) {
    throw std::runtime_error("point cloud is missing x y z fields");
  }

  split_cloud cloud_split;
  cloud_split.ground_points = laser_points;
  cloud_split.obstacle_points = laser_points;
  cloud_split.ground_points.data.clear();
  cloud_split.obstacle_points.data.clear();
  cloud_split.ground_points.data.reserve(laser_points.data.size());
  cloud_split.obstacle_points.data.reserve(laser_points.data.size());

  const std::size_t point_count =
    static_cast<std::size_t>(laser_points.width) * static_cast<std::size_t>(laser_points.height);

  std::size_t ground_count = 0;
  std::size_t obstacle_count = 0;

  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    const simple_point one_point = read_point(laser_points, point_number, point_place_data);
    if (one_point.z <= floor_height_) {
      write_point(cloud_split.ground_points, laser_points, point_number);
      ++ground_count;
    } else {
      write_point(cloud_split.obstacle_points, laser_points, point_number);
      ++obstacle_count;
    }
  }

  cloud_split.ground_points.width = static_cast<std::uint32_t>(ground_count);
  cloud_split.ground_points.height = 1u;
  cloud_split.ground_points.row_step =
    cloud_split.ground_points.point_step * cloud_split.ground_points.width;
  cloud_split.ground_points.is_dense = true;

  cloud_split.obstacle_points.width = static_cast<std::uint32_t>(obstacle_count);
  cloud_split.obstacle_points.height = 1u;
  cloud_split.obstacle_points.row_step =
    cloud_split.obstacle_points.point_step * cloud_split.obstacle_points.width;
  cloud_split.obstacle_points.is_dense = true;

  return cloud_split;
}

SensorPreprocessingCore::curve_cloud SensorPreprocessingCore::split_curve_points(
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  const point_place point_place_data = find_point_place(laser_points);
  if (!point_place_data.ready) {
    throw std::runtime_error("point cloud is missing x y z fields");
  }

  curve_cloud curve_split;
  curve_split.edge_points = laser_points;
  curve_split.flat_points = laser_points;
  curve_split.edge_points.data.clear();
  curve_split.flat_points.data.clear();
  curve_split.edge_points.data.reserve(laser_points.data.size());
  curve_split.flat_points.data.reserve(laser_points.data.size());

  const std::size_t point_count =
    static_cast<std::size_t>(laser_points.width) * static_cast<std::size_t>(laser_points.height);

  if (neighbor_size_ == 0 || point_count == 0) {
    curve_split.edge_points.width = 0u;
    curve_split.edge_points.height = 1u;
    curve_split.edge_points.row_step = 0u;
    curve_split.edge_points.is_dense = true;
    curve_split.flat_points.width = 0u;
    curve_split.flat_points.height = 1u;
    curve_split.flat_points.row_step = 0u;
    curve_split.flat_points.is_dense = true;
    return curve_split;
  }

  std::vector<simple_point> point_list;
  point_list.reserve(point_count);
  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    point_list.push_back(read_point(laser_points, point_number, point_place_data));
  }

  std::size_t edge_count = 0;
  std::size_t flat_count = 0;
  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    const double curve_score = find_curve_score(point_list, point_number);
    if (curve_score >= edge_score_limit_) {
      write_point(curve_split.edge_points, laser_points, point_number);
      ++edge_count;
    } else if (curve_score <= flat_score_limit_) {
      write_point(curve_split.flat_points, laser_points, point_number);
      ++flat_count;
    }
  }

  curve_split.edge_points.width = static_cast<std::uint32_t>(edge_count);
  curve_split.edge_points.height = 1u;
  curve_split.edge_points.row_step =
    curve_split.edge_points.point_step * curve_split.edge_points.width;
  curve_split.edge_points.is_dense = true;

  curve_split.flat_points.width = static_cast<std::uint32_t>(flat_count);
  curve_split.flat_points.height = 1u;
  curve_split.flat_points.row_step =
    curve_split.flat_points.point_step * curve_split.flat_points.width;
  curve_split.flat_points.is_dense = true;

  return curve_split;
}

vision_msgs::msg::BoundingBox3DArray SensorPreprocessingCore::find_group_boxes(
  const sensor_msgs::msg::PointCloud2 & laser_points) const
{
  if (group_gap_ <= 0.0) {
    throw std::runtime_error("group gap must be greater than zero");
  }

  const point_place point_place_data = find_point_place(laser_points);
  if (!point_place_data.ready) {
    throw std::runtime_error("point cloud is missing x y z fields");
  }

  const std::size_t point_count =
    static_cast<std::size_t>(laser_points.width) * static_cast<std::size_t>(laser_points.height);

  std::vector<simple_point> point_list;
  point_list.reserve(point_count);
  for (std::size_t point_number = 0; point_number < point_count; ++point_number) {
    point_list.push_back(read_point(laser_points, point_number, point_place_data));
  }

  std::vector<bool> used_points(point_count, false);
  vision_msgs::msg::BoundingBox3DArray group_boxes;
  group_boxes.header = laser_points.header;

  for (std::size_t start_number = 0; start_number < point_count; ++start_number) {
    if (used_points[start_number]) {
      continue;
    }

    std::deque<std::size_t> open_points;
    open_points.push_back(start_number);
    used_points[start_number] = true;

    double left_x = DBL_MAX;
    double right_x = -DBL_MAX;
    double left_y = DBL_MAX;
    double right_y = -DBL_MAX;
    double low_z = DBL_MAX;
    double high_z = -DBL_MAX;

    while (!open_points.empty()) {
      const std::size_t one_number = open_points.front();
      open_points.pop_front();
      const simple_point & one_point = point_list[one_number];

      left_x = std::min(left_x, static_cast<double>(one_point.x));
      right_x = std::max(right_x, static_cast<double>(one_point.x));
      left_y = std::min(left_y, static_cast<double>(one_point.y));
      right_y = std::max(right_y, static_cast<double>(one_point.y));
      low_z = std::min(low_z, static_cast<double>(one_point.z));
      high_z = std::max(high_z, static_cast<double>(one_point.z));

      for (std::size_t check_number = 0; check_number < point_count; ++check_number) {
        if (used_points[check_number]) {
          continue;
        }

        if (point_gap(one_point, point_list[check_number]) <= group_gap_) {
          used_points[check_number] = true;
          open_points.push_back(check_number);
        }
      }
    }

    vision_msgs::msg::BoundingBox3D one_box;
    one_box.center.position.x = (left_x + right_x) * 0.5;
    one_box.center.position.y = (left_y + right_y) * 0.5;
    one_box.center.position.z = (low_z + high_z) * 0.5;
    one_box.center.orientation.x = 0.0;
    one_box.center.orientation.y = 0.0;
    one_box.center.orientation.z = 0.0;
    one_box.center.orientation.w = 1.0;
    one_box.size.x = right_x - left_x;
    one_box.size.y = right_y - left_y;
    one_box.size.z = high_z - low_z;
    group_boxes.boxes.push_back(one_box);
  }

  return group_boxes;
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

void SensorPreprocessingCore::write_point_with_new_place(
  sensor_msgs::msg::PointCloud2 & clean_points,
  const sensor_msgs::msg::PointCloud2 & laser_points,
  std::size_t point_number,
  const point_place & point_place_data,
  const simple_point & new_place) const
{
  const std::size_t start_place = point_number * laser_points.point_step;
  const auto start_data = laser_points.data.begin() + static_cast<std::ptrdiff_t>(start_place);
  const auto end_data = start_data + static_cast<std::ptrdiff_t>(laser_points.point_step);
  const std::size_t old_size = clean_points.data.size();
  clean_points.data.insert(clean_points.data.end(), start_data, end_data);

  std::memcpy(
    &clean_points.data[old_size + point_place_data.x_place],
    &new_place.x,
    sizeof(float));
  std::memcpy(
    &clean_points.data[old_size + point_place_data.y_place],
    &new_place.y,
    sizeof(float));
  std::memcpy(
    &clean_points.data[old_size + point_place_data.z_place],
    &new_place.z,
    sizeof(float));
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

std::vector<double> SensorPreprocessingCore::turn_xyz(
  double x,
  double y,
  double z,
  const geometry_msgs::msg::Transform & frame_link) const
{
  const double turn_x = frame_link.rotation.x;
  const double turn_y = frame_link.rotation.y;
  const double turn_z = frame_link.rotation.z;
  const double turn_w = frame_link.rotation.w;

  const double row_one_one = 1.0 - 2.0 * (turn_y * turn_y + turn_z * turn_z);
  const double row_one_two = 2.0 * (turn_x * turn_y - turn_z * turn_w);
  const double row_one_three = 2.0 * (turn_x * turn_z + turn_y * turn_w);
  const double row_two_one = 2.0 * (turn_x * turn_y + turn_z * turn_w);
  const double row_two_two = 1.0 - 2.0 * (turn_x * turn_x + turn_z * turn_z);
  const double row_two_three = 2.0 * (turn_y * turn_z - turn_x * turn_w);
  const double row_three_one = 2.0 * (turn_x * turn_z - turn_y * turn_w);
  const double row_three_two = 2.0 * (turn_y * turn_z + turn_x * turn_w);
  const double row_three_three = 1.0 - 2.0 * (turn_x * turn_x + turn_y * turn_y);

  const double new_x = row_one_one * x + row_one_two * y + row_one_three * z;
  const double new_y = row_two_one * x + row_two_two * y + row_two_three * z;
  const double new_z = row_three_one * x + row_three_two * y + row_three_three * z;

  return {new_x, new_y, new_z};
}

SensorPreprocessingCore::box_number SensorPreprocessingCore::find_box_number(
  const simple_point & one_point) const
{
  box_number one_box{};
  one_box.x_box = static_cast<int>(std::floor(static_cast<double>(one_point.x) / box_size_));
  one_box.y_box = static_cast<int>(std::floor(static_cast<double>(one_point.y) / box_size_));
  one_box.z_box = static_cast<int>(std::floor(static_cast<double>(one_point.z) / box_size_));
  return one_box;
}

SensorPreprocessingCore::simple_point SensorPreprocessingCore::find_box_middle(
  const box_number & one_box) const
{
  simple_point middle_point{};
  middle_point.x = static_cast<float>((static_cast<double>(one_box.x_box) + 0.5) * box_size_);
  middle_point.y = static_cast<float>((static_cast<double>(one_box.y_box) + 0.5) * box_size_);
  middle_point.z = static_cast<float>((static_cast<double>(one_box.z_box) + 0.5) * box_size_);
  return middle_point;
}

SensorPreprocessingCore::simple_point SensorPreprocessingCore::turn_point_by_turn_speed(
  const simple_point & one_point,
  const sensor_msgs::msg::Imu & imu_data,
  double point_fraction) const
{
  const double turn_time = point_fraction * scan_time_;
  const double x_turn = -imu_data.angular_velocity.x * turn_time;
  const double y_turn = -imu_data.angular_velocity.y * turn_time;
  const double z_turn = -imu_data.angular_velocity.z * turn_time;

  const double x_sin = std::sin(x_turn);
  const double x_cos = std::cos(x_turn);
  const double y_sin = std::sin(y_turn);
  const double y_cos = std::cos(y_turn);
  const double z_sin = std::sin(z_turn);
  const double z_cos = std::cos(z_turn);

  const double first_x = static_cast<double>(one_point.x);
  const double first_y =
    static_cast<double>(one_point.y) * x_cos - static_cast<double>(one_point.z) * x_sin;
  const double first_z =
    static_cast<double>(one_point.y) * x_sin + static_cast<double>(one_point.z) * x_cos;

  const double second_x = first_x * y_cos + first_z * y_sin;
  const double second_y = first_y;
  const double second_z = -first_x * y_sin + first_z * y_cos;

  simple_point fixed_point{};
  fixed_point.x = static_cast<float>(second_x * z_cos - second_y * z_sin);
  fixed_point.y = static_cast<float>(second_x * z_sin + second_y * z_cos);
  fixed_point.z = static_cast<float>(second_z);
  return fixed_point;
}

double SensorPreprocessingCore::find_curve_score(
  const std::vector<simple_point> & point_list,
  std::size_t point_number) const
{
  if (point_list.empty()) {
    return 0.0;
  }

  if (point_number < neighbor_size_ || point_number + neighbor_size_ >= point_list.size()) {
    return 0.0;
  }

  double x_sum = 0.0;
  double y_sum = 0.0;
  double z_sum = 0.0;

  for (std::size_t neighbor_number = point_number - neighbor_size_;
    neighbor_number <= point_number + neighbor_size_;
    ++neighbor_number)
  {
    if (neighbor_number == point_number) {
      continue;
    }

    x_sum += static_cast<double>(point_list[neighbor_number].x);
    y_sum += static_cast<double>(point_list[neighbor_number].y);
    z_sum += static_cast<double>(point_list[neighbor_number].z);
  }

  const double neighbor_count = static_cast<double>(neighbor_size_ * 2);
  const double x_gap = x_sum / neighbor_count - static_cast<double>(point_list[point_number].x);
  const double y_gap = y_sum / neighbor_count - static_cast<double>(point_list[point_number].y);
  const double z_gap = z_sum / neighbor_count - static_cast<double>(point_list[point_number].z);

  return x_gap * x_gap + y_gap * y_gap + z_gap * z_gap;
}

double SensorPreprocessingCore::point_gap(
  const simple_point & first_point,
  const simple_point & second_point) const
{
  const double x_gap = static_cast<double>(first_point.x) - static_cast<double>(second_point.x);
  const double y_gap = static_cast<double>(first_point.y) - static_cast<double>(second_point.y);
  const double z_gap = static_cast<double>(first_point.z) - static_cast<double>(second_point.z);
  return std::sqrt(x_gap * x_gap + y_gap * y_gap + z_gap * z_gap);
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
