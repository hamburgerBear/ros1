#pragma once

// 控制插件、控制算法、控制状态机相互解耦
#include "control_common/control_base.h"
#include "control_common/hsm.h"

namespace control {
class FollowWallController : public PluginBase {
 public:
  FollowWallController() = default;
  ~FollowWallController() = default;

  bool init(const std::string& name,
            const DependencyInjector::Ptr& injector) override;
  void run() override;

 private:
  hsm::StateMachine stateMachine;
};
}  // namespace control