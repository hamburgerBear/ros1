#pragma once

#include "control_common/control_base.h"

namespace control {

// init
// setPlan
// computeCommandVelocity
// isGoalArrived
class Arc : public ControlBase {
 public:
  struct ArcArgs : Args {
    double rotation_distance;  //旋转弧度
    double rotation_radius;    //转弯半径
  };

  // 设置控制任务全局资源
  explicit Arc(const std::string& name,
               const DependencyInjector::Ptr& injector);
  ~Arc();

  // setGoal, 设置控制任务目标
  virtual void setGoal(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();

 private:
  double rotationRadius() const;

  std::shared_ptr<ArcArgs> args_;
};

}  // namespace control
