#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

bool FollowWallController::init(const std::string& name,
                                const DependencyInjector::Ptr& injector) {
  name_ = name;
  injector_ = injector;
  // control_map_["translation"] =
  //     std::make_shared<Translation>("translation", injector);
  // control_map_["rotation"] = std::make_shared<Rotation>("rotation",
  // injector); control_map_["arc"] = std::make_shared<Arc>("arc", injector);
  return true;
}

void FollowWallController::run() {
  // if (control_map_.find(injector_->algorithm_name_) == control_map_.end()) {
  //   ROS_WARN("Whitout this control algorithm[%s].",
  //            injector_->algorithm_name_.c_str());
  // } else {
  //   ROS_INFO("Algorithm running.");
  //   auto control = control_map_[injector_->algorithm_name_];
  //   control->Run();
  // }
}
}  // namespace control