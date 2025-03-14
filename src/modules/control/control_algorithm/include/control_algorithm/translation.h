#pragma once

#include "control_common/control_base.h"

namespace control {

class Translation : public ControlBase {
 public:
  struct TranslationArgs : Args {
    TranslationArgs(const std::vector<Eigen::Vector3d>& path,
                    const double& speed)
        : Args(path, speed) {}

    bool fixed_head;
  };

  explicit Translation(const std::string& name,
                       const DependencyInjector::Ptr& injector);
  ~Translation();

  virtual void setGoal(std::shared_ptr<Args> args);
  virtual void update();
};

}  // namespace control