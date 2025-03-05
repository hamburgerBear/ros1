#pragma once

#include "dependency_injector.h"

namespace control {

class AlgorithmBase {
 public:
  using Ptr = std::shared_ptr<AlgorithmBase>;

  struct Args {
  }

  //   enum State {
  // OK = 0;

  //   // Control module error codes start from here.
  //   CONTROL_ERROR = 1000;
  //   CONTROL_INIT_ERROR = 1001;
  //   CONTROL_COMPUTE_ERROR = 1002;
  //   CONTROL_ESTOP_ERROR = 1003;
  //   PERFECT_CONTROL_ERROR = 1004;

  //   }

  explicit AlgorithmBase(const std::string& name,
                         const DependencyInjector::Ptr& injector) {
    name_ = name;
    injector_ = injector;
  }

  virtual ~AlgorithmBase() = default;

  virtual void enter(std::shared_ptr<Args> args) = 0;
  virtual void execute() = 0;
  virtual void exit() = 0;

  std::string Name() { return name_; }

  std::string name_;
  DependencyInjector::Ptr injector_;
};

class PluginBase {
 public:
  using Ptr = std::shared_ptr<PluginBase>;

  virtual ~PluginBase() = default;

  virtual bool init(const std::string& name,
                    const DependencyInjector::Ptr& injector) = 0;
  virtual void run() = 0;
  // virtual void reset() = 0;
  std::string Name() { return name_; }

  std::string name_;
  DependencyInjector::Ptr injector_;
};

}  // namespace control