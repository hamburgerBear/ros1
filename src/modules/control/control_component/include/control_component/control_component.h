#pragma once

#include <actionlib/server/simple_action_server.h>
#include <control_interface/ControlTaskAction.h>
#include <ros/ros.h>

#include <pluginlib/class_loader.hpp>

#include "control_common/dependency_injector.h"
#include "control_plugin/base_controller.h"

namespace control {

using ControlTaskServer =
    actionlib::SimpleActionServer<control_interface::ControlTaskAction>;

class ControlComponent {
 public:
  using PluginMap = std::unordered_map<std::string, PluginBase::Ptr>;

  explicit ControlComponent();
  ~ControlComponent();

  void controlTask(const control_interface::ControlTaskGoalConstPtr& goal);

 private:
  std::shared_ptr<DependencyInjector> dependency_injector_;
  ros::NodeHandle nh_;

  std::unique_ptr<ControlTaskServer> control_task_;
  PluginMap plugin_map_;
};

}  // namespace control