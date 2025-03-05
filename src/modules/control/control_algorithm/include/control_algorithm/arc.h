#pragma once

#include "control_common/control_base.h"

namespace control {

class Arc : public AlgorithmBase {
 public:
  explicit Arc(const std::string& name,
               const DependencyInjector::Ptr& injector);
  ~Arc();

  void enter() override;
  void execute() override;
  void exit() override;
};

}  // namespace control