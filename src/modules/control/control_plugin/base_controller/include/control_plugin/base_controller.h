#pragma once

#include <unordered_map>

#include "control_common/state_machine.h"

// #include "control_common/control_base.h"

namespace control {

class BaseController : public PluginBase {
 public:
  BaseController() = default;
  ~BaseController() override = default;

  bool init(const std::string& name,
            const DependencyInjector::Ptr& injector) override;
  PluginStage run() override;

 private:
  StateMachine::Ptr state_machine_;
  DependencyInjector::Ptr injector_;
};

}  // namespace control