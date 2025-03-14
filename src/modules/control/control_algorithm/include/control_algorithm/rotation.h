#pragma once

#include "control_common/control_base.h"

namespace control {

class Rotation : public ControlBase {
 public:
  struct RotationArgs : Args {
    RotationArgs(const std::vector<Eigen::Vector3d>& path, const double& speed)
        : Args(path, speed) {}
  };

  explicit Rotation(const std::string& name,
                    const DependencyInjector::Ptr& injector);
  ~Rotation();

  virtual void setGoal(std::shared_ptr<Args> args);
  virtual void update();
};

}  // namespace control