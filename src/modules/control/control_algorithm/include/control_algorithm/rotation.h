#pragma once

#include "control_common/control_base.h"

namespace control {

class Rotation : public ControlBase {
 public:
  struct RotationArgs : Args {
    double rotation_distance;  //旋转弧度
  };

  explicit Rotation(const std::string& name,
                    const DependencyInjector::Ptr& injector);
  ~Rotation();

  virtual void setGoal(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();

 private:
  std::shared_ptr<RotationArgs> args_;
};

}  // namespace control