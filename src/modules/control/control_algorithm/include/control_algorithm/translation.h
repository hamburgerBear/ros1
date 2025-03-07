#pragma once

#include "control_common/control_base.h"

namespace control {

class Translation : public ControlBase {
 public:
  struct Args {
    double distance;
    double velocity;
  };

  explicit Translation(const std::string& name,
                       const DependencyInjector::Ptr& injector);
  ~Translation();

  virtual void init(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();

 private:
};

}  // namespace control