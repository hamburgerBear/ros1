#include "control_common/utils.h"

namespace control {

double sign(double value) {
  if (value > 0.0)
    return 1.0;
  else
    return -1.0;
}

unsigned int to360(int angle) {
  while (angle < 0) angle += 360;
  while (angle > 359) angle -= 360;
  return static_cast<unsigned int>(angle);
};

int to180(unsigned int angle) {
  angle = static_cast<int>(angle);
  while (angle > 180) angle -= 360;

  return angle;
}

double toRad(double deg) { return angles::from_degrees(deg); }

double toDeg(double rad) { return angles::to_degrees(rad); }

void getOdomTFAndTwist(const ros::Time& time,
                       const std::deque<nav_msgs::OdometryPtr>& odom_deque,
                       Eigen::Isometry3d& odom_to_baselink,
                       geometry_msgs::Twist& twist) {
  auto odom_temp = odom_deque.front();
  for (auto it = odom_deque.rbegin(); it != odom_deque.rend(); ++it) {
    const nav_msgs::OdometryPtr& odom = *it;
    if (time > odom->header.stamp) {
      odom_temp = *it;
      double dt = (time - odom_temp->header.stamp).toSec();
      double dangle = odom_temp->twist.twist.angular.z * dt;
      tf2::Quaternion q(odom_temp->pose.pose.orientation.x,
                        odom_temp->pose.pose.orientation.y,
                        odom_temp->pose.pose.orientation.z,
                        odom_temp->pose.pose.orientation.w);
      geometry_msgs::Quaternion quat_msg;
      quat_msg.x = q.x();
      quat_msg.y = q.y();
      quat_msg.z = q.z();
      quat_msg.w = q.w();
      odom_temp->pose.pose.orientation = quat_msg;
      break;
    }
  }

  twist = odom_temp->twist.twist;
  // 从里程计消息中提取位置和方向
  const geometry_msgs::Pose& pose = odom_temp->pose.pose;
  const geometry_msgs::Point& position = pose.position;
  const geometry_msgs::Quaternion& orientation = pose.orientation;

  // 将四元数转换为 Eigen 旋转矩阵
  Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y,
                          orientation.z);
  Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

  // 创建一个 Eigen::Isometry3d 变换
  odom_to_baselink = Eigen::Isometry3d::Identity();
  odom_to_baselink.linear() = rotation_matrix;
  odom_to_baselink.translation() =
      Eigen::Vector3d(position.x, position.y, position.z);

  if (fabs((time - odom_temp->header.stamp).toSec()) > 0.5) {
    // std::cout << "里程计超时了 = "
    //           << fabs((time - odom_temp.header.stamp).toSec()) <<
    //           std::endl;
  }
}

bool toBaselink(const sensor_msgs::LaserScanPtr& sensor,
                const Eigen::Isometry3d& baselink_to_sensor,
                const std::deque<nav_msgs::OdometryPtr>& odom_deque,
                PointCloud& points_at_base) {
  if (sensor == nullptr) {
    ROS_WARN_THROTTLE(1.0, "Sensor data is nullptr, please check sensor.");
    return false;
  }

  double dt = (ros::Time::now() - sensor->header.stamp).toSec();
  if (dt >= 3.0) {
    ROS_WARN_THROTTLE(1.0,
                      "Sensor(%s) data is timeout %fs, please check sensor.",
                      sensor->header.frame_id.c_str(), dt);
    return false;
  }

  if (sensor->ranges.size() <= 0) {
    ROS_WARN_THROTTLE(1.0, "Sensor(%s) data is empty, please check sensor.",
                      sensor->header.frame_id.c_str());
    return false;
  }

  auto start_time = ros::Time::now();
  std::vector<Eigen::Vector2d> points;
  double x, y, theta, range;
  for (size_t i = 0; i < sensor->ranges.size(); ++i) {
    double range = sensor->ranges[i];
    if (std::isinf(sensor->ranges[i]) ||
        sensor->ranges[i] <= sensor->range_min ||
        std::isnan(sensor->ranges[i])) {
      range = sensor->range_max;
    }

    theta = sensor->angle_min + i * sensor->angle_increment;
    x = range * cos(theta);
    y = range * sin(theta);
    points.emplace_back(x, y);
  }

  //硬时间同步
  //去除运动畸变
  //感兴趣区域过滤
  auto filtered_points = points;
  //多传感器时间同步
  //坐标系转换(假设水平面安装的高效坐标系转换)

  PointCloud empty;
  std::swap(empty, points_at_base);
  Eigen::Vector3d point_at_sensor, point_at_base;

  Eigen::Isometry3d odom_to_baselink_t0, odom_to_baselink_t1;
  geometry_msgs::Twist twist;
  getOdomTFAndTwist(sensor->header.stamp, odom_deque, odom_to_baselink_t0,
                    twist);
  getOdomTFAndTwist(ros::Time::now(), odom_deque, odom_to_baselink_t1, twist);
  Eigen::Isometry3d T_combined =
      odom_to_baselink_t1.inverse() * odom_to_baselink_t0 * baselink_to_sensor;

  for (const auto& point : filtered_points) {
    point_at_sensor << point(0), point(1), 0.0;
    // point_at_base = baselink_to_sensor * point_at_sensor;
    point_at_base = T_combined * point_at_sensor;
    points_at_base.emplace_back(point_at_base);
  }

  dt = (ros::Time::now() - start_time).toSec() * 1000.0;
  if (dt > 10)
    ROS_WARN("Sensor(%s) to baselink timeout %fms",
             sensor->header.frame_id.c_str(), dt);
  else
    ROS_INFO_THROTTLE(30.0, "Sensor(%s) to baselink cost %fms",
                      sensor->header.frame_id.c_str(), dt);

  return true;
}

void discretePointcloud(const PointCloud& points_at_base,
                        DiscretePointCloud& discrete_points_at_base) {
  discrete_points_at_base.fill(std::make_pair(
      INVALID_VALUE, Eigen::Vector2d(INVALID_VALUE, INVALID_VALUE)));
  double theta, range;
  for (const auto& point_at_base : points_at_base) {
    theta = atan2(point_at_base.y(), point_at_base.x());
    range = hypot(point_at_base.x(), point_at_base.y());
    int angle180 = round(toDeg(theta));
    unsigned int angle360 = to360(angle180);
    if (range < discrete_points_at_base[angle360].first) {
      std::pair<double, Eigen::Vector2d> pair(
          range, Eigen::Vector2d(point_at_base.x(), point_at_base.y()));
      discrete_points_at_base[angle360] = pair;
    }
  }
}

}  // namespace control