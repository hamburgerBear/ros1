#pragma once

#include <unordered_map>

#include "control_common/control_base.h"

namespace control {

class BaseController : public PluginBase {
 public:
  using ControlMap = std::unordered_map<std::string, AlgorithmBase::Ptr>;

  BaseController() = default;
  ~BaseController() override = default;

  bool init(const std::string& name,
            const DependencyInjector::Ptr& injector) override;
  void run() override;
  // void reset() override;

 private:
  ControlMap control_map_;
};

}  // namespace control