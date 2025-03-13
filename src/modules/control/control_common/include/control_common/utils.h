#pragma once

#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include </usr/include/eigen3/Eigen/Eigen>  //TODO:this
#include <array>
#include <deque>
#include <vector>

namespace control {

#define INVALID_VALUE 1e6
using PointCloud = std::vector<Eigen::Vector3d>;
using DiscretePointCloud = std::array<std::pair<double, Eigen::Vector2d>, 360>;

double sign(double value);
//角度
unsigned int to360(int angle);
int to180(unsigned int angle);
double toRad(double deg);
double toDeg(double rad);
double shortestAngularDistance(double from, double to);
double normalizeAngle(double angle);
//格式
Eigen::Vector3d toEigen(const nav_msgs::Odometry& odom);
Eigen::Vector3d toEigen(
    const geometry_msgs::PoseWithCovarianceStamped& current_pose);
//坐标系&滤波
void getOdomTFAndTwist(const ros::Time& time,
                       const std::deque<nav_msgs::OdometryPtr>& odom_deque,
                       Eigen::Isometry3d& odom_to_baselink,
                       geometry_msgs::Twist& twist);
bool toBaselink(const sensor_msgs::LaserScanPtr& sensor,
                const Eigen::Isometry3d& baselink_to_sensor,
                const std::deque<nav_msgs::OdometryPtr>& odom_deque,
                PointCloud& points_at_base);
void discretePointcloud(const PointCloud& points_at_base,
                        DiscretePointCloud& discrete_points_at_base);

}  // namespace control
