#include "control_component/control_component.h"

namespace control {
ControlComponent::ControlComponent() {
  dependency_injector_ = std::make_shared<DependencyInjector>();

  //注册插件，可以根据.yaml文件
  plugin_map_["base_controller"] = std::make_shared<BaseController>();
  plugin_map_["base_controller"]->init("base_controller", dependency_injector_);

  //启动控制任务action server
  control_task_ = std::make_unique<ControlTaskServer>(
      ros::NodeHandle(), "control_task",
      boost::bind(&ControlComponent::controlTask, this, _1), false);
  control_task_->start();
}

ControlComponent::~ControlComponent() {}

void ControlComponent::controlTask(
    const control_interface::ControlTaskGoalConstPtr& goal) {
  ROS_INFO("Task start that plugin_name[%s], algorithm_name[%s].",
           goal->plugin_name.c_str(), goal->algorithm_name.c_str());

  if (plugin_map_.find(goal->plugin_name) == plugin_map_.end()) {
    ROS_WARN("Task aborted, because invaild plugin name[%s].",
             goal->plugin_name.c_str());
    control_task_->setAborted(control_interface::ControlTaskResult(),
                              "Aborted task.");
    return;
  }

  dependency_injector_->plugin_name_ = goal->plugin_name;
  dependency_injector_->algorithm_name_ = goal->algorithm_name;

  auto controller = plugin_map_[goal->plugin_name];
  controller->run();

  control_task_->setSucceeded(control_interface::ControlTaskResult(),
                              "Succeeded task.");
  ROS_INFO("Task succeeded.");
  return;
}

}  // namespace control