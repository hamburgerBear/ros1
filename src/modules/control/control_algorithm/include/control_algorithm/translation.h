#pragma once

#include "control_common/control_base.h"

namespace control {

class Translation : public AlgorithmBase {
 public:
  struct Args {
    double distance;
    double velocity;
  };

  explicit Translation(const std::string& name,
                       const DependencyInjector::Ptr& injector);
  ~Translation();

  void enter() override;
  void execute() override;
  void exit() override;

  // void setGoal(std::shared_ptr<Args> goal);

 private:
  //任务目标
  std::shared_ptr<Args> goal_;

  //算法参数

  //
};

}  // namespace control