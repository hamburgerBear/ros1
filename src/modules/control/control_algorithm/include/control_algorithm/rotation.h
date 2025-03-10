#pragma once

#include "control_common/control_base.h"

namespace control {

class Rotation : public ControlBase {
 public:
  struct RotationArgs : Args {
    double rotation_theta;         //旋转弧度
    double max_angular_velocity;   //最大角速度
    double min_angular_velocityl;  //最小角速度
    double acc_angular_velocity;   //角加速度
  };

  explicit Rotation(const std::string& name,
                    const DependencyInjector::Ptr& injector);
  ~Rotation();

  virtual void init(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();
};

}  // namespace control