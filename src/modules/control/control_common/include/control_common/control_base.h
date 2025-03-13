#pragma once

#include "dependency_injector.h"
#include "utils.h"

namespace control {

class ControlBase {
 public:
  struct Args {
    virtual ~Args() {}
    double max_linear_velocity;   //最大线速度
    double min_linear_velocity;   //最小线速度
    double max_angular_velocity;  //最大角速度
    double min_angular_velocity;  //最小角速度
    double acc_linear_velocity;   //线加速度
    double dcc_linear_velocity;   //线减速度
    double acc_angular_velocity;  //角加速度
    double timeout_threshold;
  };

  using Ptr = std::shared_ptr<ControlBase>;
  using Transition = std::pair<std::string, std::shared_ptr<ControlBase::Args>>;

  explicit ControlBase(const std::string& name,
                       const DependencyInjector::Ptr& injector)
      : name_(name), injector_(injector) {}
  virtual ~ControlBase() = default;

  virtual void setGoal(std::shared_ptr<Args> args) = 0;
  virtual void update() = 0;
  virtual bool isFinish() = 0;
  virtual bool isFail() = 0;
  virtual std::pair<std::string, std::shared_ptr<ControlBase::Args>>
  transition() {
    return std::make_pair(name(), nullptr);
  };

  std::string name() { return name_; }
  DependencyInjector::Ptr injector() { return injector_; }

 public:
  // protected:
  double desiredDistance() const { return desired_distance_; }
  double accumulationDistance() const { return accumulated_distance_; }

  Eigen::Vector3d start_pose_;   //控制任务的起点
  Eigen::Vector3d last_pose_;    //上一时刻的位置
  ros::Time start_time_;         //控制任务的开始时间
  double desired_distance_;      //期望的控制距离
  double accumulated_distance_;  //实际累积的控制距离

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
  DependencyInjector::Ptr injector() { return injector_; }

  std::string name_;
  DependencyInjector::Ptr injector_;
};

}  // namespace control