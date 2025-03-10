#pragma once

#include "control_common/control_base.h"

namespace control {

class Arc : public ControlBase {
 public:
  struct ArcArgs : Args {
    double rotation_theta;         //旋转弧度
    double rotation_radius;        //转弯半径
    double max_linear_velocity;    //最大线速度
    double min_linear_velocity;    //最小线速度
    double max_angular_velocity;   //最大角速度
    double min_angular_velocityl;  //最小角速度
    double acc_linear_velocity;    //线加速度
    double dcc_linear_velocity;    //线减速度
    double acc_angular_velocity;   //角加速度
  };

  // 设置控制任务全局资源
  explicit Arc(const std::string& name,
               const DependencyInjector::Ptr& injector);
  ~Arc();

  // setGoal, 设置控制任务目标
  virtual void init(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();
};

}  // namespace control
