#pragma once

#include "control_common/control_base.h"

namespace control {

class Rotation : public ControlBase {
 public:
  explicit Rotation(const std::string& name,
                    const DependencyInjector::Ptr& injector);
  ~Rotation();

  virtual void init(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();
};

}  // namespace control