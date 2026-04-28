# sensor_preprocessing_suite

`sensor_preprocessing_suite` is a ROS 2 C++ library and component package for preparing raw IMU and LiDAR data before downstream localization, mapping, or perception.

## Features

- Time synchronization between `sensor_msgs/msg/Imu` and `sensor_msgs/msg/PointCloud2`
- Gravity removal from IMU linear acceleration
- TF-based rotation from IMU frame into LiDAR frame
- Point cloud range filtering
- Motion deskewing using IMU turn speed
- Voxel box downsampling
- Ground and obstacle separation using a height split
- Edge and flat feature extraction using local curve score
- Obstacle grouping into 3D boxes published as `vision_msgs/msg/BoundingBox3DArray`

## Package Layout

- `include/sensor_preprocessing_suite/sensor_preprocessing_core.hpp`
- `include/sensor_preprocessing_suite/sensor_preprocessing_component.hpp`
- `src/sensor_preprocessing_core.cpp`
- `src/sensor_preprocessing_component.cpp`
- `config/default_params.yaml`

## Topics

### Subscriptions

- `imu/in` as `sensor_msgs/msg/Imu`
- `points/in` as `sensor_msgs/msg/PointCloud2`

### Publications

- `imu/out` as `sensor_msgs/msg/Imu`
- `points/out` as `sensor_msgs/msg/PointCloud2`
- `points/ground` as `sensor_msgs/msg/PointCloud2`
- `points/obstacle` as `sensor_msgs/msg/PointCloud2`
- `points/edge` as `sensor_msgs/msg/PointCloud2`
- `points/flat` as `sensor_msgs/msg/PointCloud2`
- `points/group_boxes` as `vision_msgs/msg/BoundingBox3DArray`

## Parameters

- `time_limit`
- `near_limit`
- `far_limit`
- `box_size`
- `floor_height`
- `scan_time`
- `neighbor_size`
- `edge_score_limit`
- `flat_score_limit`
- `group_gap`
- `imu_frame`
- `lidar_frame`

The default values are provided in [`config/default_params.yaml`](./config/default_params.yaml).

## Build

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select sensor_preprocessing_suite
```

Then source the workspace:

```bash
source install/setup.bash
```

## Run As A Component

Example standalone container:

```bash
ros2 run rclcpp_components component_container
```

In another terminal:

```bash
ros2 component load /ComponentManager sensor_preprocessing_suite sensor_preprocessing_suite::SensorPreprocessingComponent --param-file install/sensor_preprocessing_suite/share/sensor_preprocessing_suite/config/default_params.yaml
```

## Processing Flow

1. IMU and LiDAR messages are synchronized.
2. Gravity is removed from the IMU acceleration.
3. IMU data is rotated into the LiDAR frame.
4. The point cloud is range filtered.
5. The cloud is deskewed from IMU turn speed.
6. The cloud is downsampled.
7. Ground and obstacle clouds are separated.
8. Edge and flat feature clouds are extracted.
9. Obstacle points are grouped into 3D boxes.

## Notes

- The deskew model assumes points later in the cloud were captured later in the scan.
- The grouping step uses a simple distance-based region growing pass.
- The edge and flat split depends on the order of points in the incoming cloud.

## Dependencies

- `rclcpp`
- `rclcpp_components`
- `sensor_msgs`
- `message_filters`
- `geometry_msgs`
- `tf2`
- `tf2_ros`
- `vision_msgs`

## License

`Apache-2.0`
