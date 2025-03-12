#pragma once

#include <actionlib/server/simple_action_server.h>  //ActionServer
#include <control_interface/ControlTaskAction.h>    //ActionMsg
#include <ros/ros.h>                                //ROS

#include <pluginlib/class_loader.hpp>  //Plugin

#include "control_common/dependency_injector.h"
// #include "control_plugin/base_controller.h"
#include <unordered_map>

#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

using ControlTaskServer =
    actionlib::SimpleActionServer<control_interface::ControlTaskAction>;

class ControlComponent {
 public:
  using PluginMap = std::unordered_map<std::string, PluginBase::Ptr>;

  explicit ControlComponent();
  ~ControlComponent();

  void controlTask(const control_interface::ControlTaskGoalConstPtr& goal);

  void scanCB(const sensor_msgs::LaserScanPtr& msg);
  void stageScanCB(const sensor_msgs::LaserScanPtr& msg);
  void poseCB(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);
  void stagePoseCB(const nav_msgs::OdometryPtr& msg);
  void odomCB(const nav_msgs::OdometryPtr& msg);
  void stageBumperCB(const std_msgs::ByteMultiArrayPtr& msg);
  void staticTfCB(const tf2_msgs::TFMessage::ConstPtr& msg);

 private:
  std::shared_ptr<DependencyInjector> dependency_injector_;
  ros::NodeHandle nh_;

  std::unique_ptr<ControlTaskServer> control_task_;
  PluginMap plugin_map_;

  // ROS数据订阅
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_stage_scan_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_stage_pose_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_stage_bumper_;
  ros::Subscriber sub_static_tf_;
};

}  // namespace control
