#pragma once

#include "dependency_injector.h"

namespace control {

class ControlBase {
 public:
  struct Args {};

  using Ptr = std::shared_ptr<ControlBase>;
  using Transition = std::pair<std::string, std::shared_ptr<ControlBase::Args>>;

  explicit ControlBase(const std::string& name,
                       const DependencyInjector::Ptr& injector)
      : name_(name), injector_(injector) {}
  virtual ~ControlBase() = default;

  virtual void init(std::shared_ptr<Args> args) = 0;
  virtual void update() = 0;
  virtual bool isFinish() = 0;
  virtual bool isFail() = 0;
  virtual std::pair<std::string, std::shared_ptr<ControlBase::Args>>
  transition() {
    return std::make_pair(name(), nullptr);
  };

  std::string name() { return name_; }
  DependencyInjector::Ptr injector() { return injector_; }

 private:
  std::string name_;
  DependencyInjector::Ptr injector_;
  Args goal_;
};

struct PluginStage {
  enum class Stage { RUNNING, PAUSED, SUCCEEDED, ABORTED };
  PluginStage() : stage(Stage::RUNNING) {}
  bool working() { return (stage <= Stage::PAUSED); }
  bool running() { return (stage == Stage::RUNNING); }
  bool paused() { return (stage == Stage::PAUSED); }
  bool succeeded() { return (stage == Stage::SUCCEEDED); }
  bool aborted() { return (stage == Stage::ABORTED); }
  Stage stage;
};

class PluginBase {
 public:
  using Ptr = std::shared_ptr<PluginBase>;

  virtual ~PluginBase() = default;

  virtual bool init(const std::string& name,
                    const DependencyInjector::Ptr& injector) = 0;
  virtual PluginStage run() = 0;
  // virtual void reset() = 0;
  std::string Name() { return name_; }

  std::string name_;
  DependencyInjector::Ptr injector_;
};

}  // namespace control