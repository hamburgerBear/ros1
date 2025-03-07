#pragma once

#include "control_common/control_base.h"

namespace control {

class Arc : public ControlBase {
 public:
  explicit Arc(const std::string& name,
               const DependencyInjector::Ptr& injector);
  ~Arc();

  virtual void init(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();
};

}  // namespace control
