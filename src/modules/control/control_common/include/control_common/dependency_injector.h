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

// enum Bumper {
//   LEFT = 0; LEFT_FRONT = 1; FRONT = 2; RIGHT_FRONT = 3; RIGHT = 4;
// };

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
  void velocity(const double& v, const double w) {
    cmd_vel_.linear.x = v;
    cmd_vel_.angular.z = w;
    last_vel.linear.x = v;
    last_vel.angular.z = w;
  }

  void velocity(const geometry_msgs::Twist& twist) {
    velocity(twist.linear.x, twist.angular.z);
  }

  geometry_msgs::Twist lastVel() { return last_vel; }
  bool collision() const {
    for (size_t i = 0; i < bumper_->data.size(); ++i)
      if (bumper_->data[i]) {
        ROS_WARN("happend collision.");
        return true;
      }
    return false;
  }

  std::string plugin_name_;
  std::string algorithm_name_;

  sensor_msgs::LaserScanPtr scan_;
  geometry_msgs::PoseWithCovarianceStampedPtr current_pose_;
  nav_msgs::OdometryPtr odom_;
  std::deque<nav_msgs::OdometryPtr> odom_deque_;
  std_msgs::ByteMultiArrayPtr bumper_;
  tf2_msgs::TFMessage::ConstPtr static_tf_;
  geometry_msgs::Twist cmd_vel_, last_vel;
};

}  // namespace control
