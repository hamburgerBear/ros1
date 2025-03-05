#pragma once

#include "control_common/control_base.h"

namespace control {

class Rotation : public AlgorithmBase {
 public:
  explicit Rotation(const std::string& name,
                    const DependencyInjector::Ptr& injector);
  ~Rotation();

  void enter() override;
  void execute() override;
  void exit() override;
};

}  // namespace control