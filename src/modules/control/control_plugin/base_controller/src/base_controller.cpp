#include "control_plugin/base_controller.h"

#include <pluginlib/class_list_macros.h>

#include "control_algorithm/arc.h"
#include "control_algorithm/rotation.h"
#include "control_algorithm/translation.h"

PLUGINLIB_EXPORT_CLASS(control::BaseController, control::PluginBase)

namespace control {

bool BaseController::init(const std::string& name,
                          const DependencyInjector::Ptr& injector) {
  name_ = name;
  injector_ = injector;
  // control_map_["translation"] =
  //     std::make_shared<Translation>("translation", injector);
  // control_map_["rotation"] = std::make_shared<Rotation>("rotation",
  // injector); control_map_["arc"] = std::make_shared<Arc>("arc", injector);
  return true;
}

void BaseController::run() {
  // if (control_map_.find(injector_->algorithm_name_) == control_map_.end()) {
  //   ROS_WARN("Whitout this control algorithm[%s].",
  //            injector_->algorithm_name_.c_str());
  // } else {
  //   ROS_INFO("Algorithm running.");
  //   auto control = control_map_[injector_->algorithm_name_];
  //   control->Run();
  // }
}

// void BaseController::reset() { control_map_.clear(); }

}  // namespace control