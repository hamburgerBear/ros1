#pragma once

#include "control_common/control_base.h"

namespace control {

class Arc : public ControlBase {
 public:
  struct ArcArgs : Args {
    ArcArgs(const std::vector<Eigen::Vector3d>& path, const double& speed,
            const double& radius)
        : Args(path, speed), rotation_radius(radius) {}
    double rotation_radius;
  };

  explicit Arc(const std::string& name,
               const DependencyInjector::Ptr& injector);
  ~Arc();

  virtual void setGoal(std::shared_ptr<Args> args);
  virtual void update();
  double rotationRadius() const;
};

}  // namespace control
