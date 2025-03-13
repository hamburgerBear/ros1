#pragma once

#include "control_common/control_base.h"

namespace control {

class Translation : public ControlBase {
 public:
  struct TranslationArgs : Args {
    double translation_distance;  //移动距离
    bool fixed_head;              //固定航向
  };

  explicit Translation(const std::string& name,
                       const DependencyInjector::Ptr& injector);
  ~Translation();

  virtual void setGoal(std::shared_ptr<Args> args);
  virtual void update();
  virtual bool isFinish();
  virtual bool isFail();

 private:
  std::shared_ptr<TranslationArgs> args_;
};

}  // namespace control