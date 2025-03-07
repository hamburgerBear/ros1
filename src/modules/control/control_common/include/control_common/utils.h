#pragma once

#include <sensor_msgs/LaserScan.h>

#include <Eigen/Eigen>
#include <arrary>
#include <vector>

namespace control {

double sign(double value) {
  if (value > 0.0)
    return 1.0;
  else
    return -1.0;
}

// 将传感器转换到BaseLink系
using DispersedCloud = std::array<std::pair<double, Eigen::Vector2d>, 360>;

DispersedCloud sensorToBaselink(
    const sensor_msgs::LaserScanConstPtr& origin_scan,
    const Eigen::Isometry3d& baselink_to_sensor) {
  DispersedCloud cloud;

  size_t size = origin_scan->ranges.size();
  return std::move(cloud);
}

void DataManager::CloudToBaseLinkFast(
    const sensor_msgs::LaserScanConstPtr& origin_scan,
    const Eigen::Isometry3d& baselink_to_sensor, DispersePointCloud& cloud) {
  if (origin_scan == nullptr) {
    ALG_WARN_THROTTLE(1.0, "Data is nullptr, please check sensor.");
    return;
  }

  double dt = (ros::Time::now() - origin_scan->header.stamp).toSec();
  if (dt >= 3.0) {
    ALG_WARN_THROTTLE(1.0, "Data %s is timeout %fs, please check sensor.",
                      origin_scan->header.frame_id, dt);
    return;
  }

  if (origin_scan->ranges.size() <= 0) {
    ALG_WARN_THROTTLE(1.0, "Without %s data, please check sensor.",
                      origin_scan->header.frame_id);
    return;
  }

  /* 之前排查过这个地方，O队列 */
  ros::Time start_time = ros::Time::now();
  double interest_range = 1.0f;
  double interest_length = 0.005f;
  Eigen::Isometry3d empty;
  geometry_msgs::Twist twist;
  GetOdomTFAndTwist(origin_scan->header.stamp, empty, twist);
  double linear_x = -twist.linear.x;
  double linear_y = -twist.linear.y;
  double angular_z = -twist.angular.z;
  Eigen::Isometry3d odom_to_baselink_t0, odom_to_baselink_t1;
  GetOdomTFAndTwist(origin_scan->header.stamp, odom_to_baselink_t0, twist);
  GetOdomTFAndTwist(ros::Time::now(), odom_to_baselink_t1, twist);
  Eigen::Isometry3d T_combined =
      odom_to_baselink_t1.inverse() * odom_to_baselink_t0 * baselink_to_sensor;

  double theta, range;
  Eigen::Vector2d point_2d, last_point_2d = Eigen::Vector2d(0.0, 0.0);
  Eigen::Vector2d undistortion_point_2d;
  Eigen::Vector3d point_at_sensor, point_at_base;
  size_t size = origin_scan->ranges.size();
  for (size_t i = 0; i < size; ++i) {
    double range = origin_scan->ranges[i];
    if (std::isinf(origin_scan->ranges[i]) || origin_scan->ranges[i] == 0.0 ||
        std::isnan(origin_scan->ranges[i]))
      continue;

    if (range < interest_range) {
      theta = origin_scan->angle_min + i * origin_scan->angle_increment;
      point_2d.x() = range * cos(theta);
      point_2d.y() = range * sin(theta);
      double length = (point_2d - last_point_2d).norm();
      if (length < interest_length) continue;

      last_point_2d = point_2d;
      Eigen::Matrix<double, 2, 1> t = {
          linear_x * (size - 1 - i) * origin_scan->time_increment,
          linear_y * (size - 1 - i) * origin_scan->time_increment};
      Eigen::Rotation2D<double> r(angular_z * origin_scan->time_increment *
                                  (size - 1 - i));
      undistortion_point_2d = r * point_2d + t;
      point_at_sensor.x() = undistortion_point_2d.x();
      point_at_sensor.y() = undistortion_point_2d.y();
      point_at_sensor.z() = 0.0;
      point_at_base = T_combined * point_at_sensor;
      theta = atan2(point_at_base.y(), point_at_base.x());
      range = hypot(point_at_base.x(), point_at_base.y());
      int disperse_pi_index = round(angles::to_degrees(theta));
      unsigned int disperse_2pi_index = To2PI(disperse_pi_index);
      if (range < cloud[disperse_2pi_index].first) {
        std::pair<double, Eigen::Vector2d> pair(
            range, Eigen::Vector2d(point_at_base.x(), point_at_base.y()));
        cloud[disperse_2pi_index] = pair;
      }
    }
  }
}
}  // namespace control