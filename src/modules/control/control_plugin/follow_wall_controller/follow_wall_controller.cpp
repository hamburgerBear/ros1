#include "follow_wall_controller.h"

#include <pluginlib/class_list_macros.h>

#include "states/state_root.h"

PLUGINLIB_EXPORT_CLASS(control::FollowWallController, control::PluginBase)

namespace control {

bool FollowWallController::init(const std::string& name,
                                const DependencyInjector::Ptr& injector) {
  name_ = name;
  injector_ = injector;
  state_machine_ = std::make_shared<hsm::StateMachine>();
  state_machine_->Initialize<StateRoot>(this);
  return true;
}

void FollowWallController::run() {
  state_machine_->ProcessStateTransitions();
  state_machine_->UpdateStates();
  sleep(1);  // TODO:check this
}

}  // namespace control
