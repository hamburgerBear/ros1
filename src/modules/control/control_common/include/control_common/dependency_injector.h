#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ByteMultiArray.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>

namespace control {
class DependencyInjector {
 public:
  using Ptr = std::shared_ptr<DependencyInjector>;

  DependencyInjector() {
    scan_ = boost::make_shared<sensor_msgs::LaserScan>();
    current_pose_ =
        boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    odom_ = boost::make_shared<nav_msgs::Odometry>();
    bumper_ = boost::make_shared<std_msgs::ByteMultiArray>();
    // static_tf_ = boost::make_shared<tf2_msgs::TFMessagePtr>();
  };
  ~DependencyInjector() = default;

  std::string plugin_name_;
  std::string algorithm_name_;

  sensor_msgs::LaserScanPtr scan_;
  geometry_msgs::PoseWithCovarianceStampedPtr current_pose_;
  nav_msgs::OdometryPtr odom_;
  std::deque<nav_msgs::OdometryPtr> odom_deque_;
  std_msgs::ByteMultiArrayPtr bumper_;
  tf2_msgs::TFMessage::ConstPtr static_tf_;
  geometry_msgs::Twist cmd_vel_;
};

}  // namespace control
