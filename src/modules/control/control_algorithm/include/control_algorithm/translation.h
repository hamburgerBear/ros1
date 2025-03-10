#pragma once

#include "control_common/control_base.h"

namespace control {

class Translation : public ControlBase {
 public:
  struct TranslationArgs : Args {
    double translation_distance;  //移动距离
    double max_linear_velocity;   //最大线速度
    double min_linear_velocity;   //最小线速度
    double max_angular_velocity;  //最大角速度
    double acc_linear_velocity;   //线加速度
    double dcc_linear_velocity;   //线减速度
    double acc_angular_velocity;  //角加速度
    bool fixed_head;              //固定航向
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