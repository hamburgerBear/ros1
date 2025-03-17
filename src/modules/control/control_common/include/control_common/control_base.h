#pragma once

#include "dependency_injector.h"
#include "utils.h"

namespace control {

class ControlBase {
 public:
  struct Args {
    Args(const std::vector<Eigen::Vector3d>& path, const double& speed)
        : target_path(path),
          target_speed(speed),
          max_vel_x(10.0),
          min_vel_x(0.05),
          max_vel_theta(10.0),
          min_vel_theta(0.3),
          max_acc_x(10.0),
          max_dcc_x(10.0),
          max_acc_theta(10.0) {}
    virtual ~Args() {}

    std::vector<Eigen::Vector3d> target_path;  //目标路径
    double target_speed;                       //目标速度
    double max_vel_x;                          //最大线速度
    double min_vel_x;                          //最小线速度
    double max_vel_theta;                      //最大角速度
    double min_vel_theta;                      //最小角速度
    double max_acc_x;                          //最大线加速度
    double max_dcc_x;                          //最大线减速度
    double max_acc_theta;                      //最大角加速度
  };

  using Ptr = std::shared_ptr<ControlBase>;
  using Transition = std::pair<std::string, std::shared_ptr<ControlBase::Args>>;

  explicit ControlBase(const std::string& name,
                       const DependencyInjector::Ptr& injector)
      : name_(name), injector_(injector) {}
  virtual ~ControlBase() = default;

  virtual void setGoal(std::shared_ptr<Args> args) = 0;
  virtual void update() = 0;

  virtual bool isFinish() {
    if (accumulated_distance_ >= fabs(targetDistance())) {
      ROS_INFO(
          "Control(%s) finish, desired_distance(%f) accumulation_distance(%f)",
          name().c_str(), fabs(targetDistance()), accumulated_distance_);
      return true;
    } else
      return false;
  }

  virtual bool isFail() {
    double dt = (ros::Time::now() - start_time_).toSec();
    if (dt >= timeout_threshold_) {
      ROS_INFO("Control(%s) fail, timeout_threshold(%f) cost_time(%f)",
               name_.c_str(), timeout_threshold_, dt);
      return true;
    } else {
      return false;
    }
  }

  virtual std::pair<std::string, std::shared_ptr<ControlBase::Args>>
  transition() {
    return std::make_pair(name(), nullptr);
  };

 protected:
  std::string name() { return name_; }
  DependencyInjector::Ptr injector() { return injector_; }
  std::vector<Eigen::Vector3d> targetPath() const { return args_->target_path; }
  double targetDistance() const { return args_->target_path[0].x(); }
  double targetSpeed() const { return args_->target_speed; }
  double maxVelX() const { return args_->max_vel_x; }
  double minVelX() const { return args_->min_vel_x; }
  double maxVelTheta() const { return args_->max_vel_theta; }
  double minVelTheta() const { return args_->min_vel_theta; }
  double MaxAccX() const { return args_->max_acc_x; }
  double MaxDccX() const { return args_->max_dcc_x; }
  double MaxAccTheta() const { return args_->max_acc_theta; }
  Eigen::Vector3d startPose() const { return start_pose_; }
  Eigen::Vector3d lastPose() const { return last_pose_; }
  ros::Time startTime() const { return start_time_; }
  double accumulationDistance() const { return accumulated_distance_; }
  double timeoutThreshold() const { return timeout_threshold_; }

  std::shared_ptr<Args> args_;   //目标参数，由外部提供
                                 //变量，有算法内部计算和保存
  Eigen::Vector3d start_pose_;   //控制任务的起点
  Eigen::Vector3d last_pose_;    //上一时刻的位置
  ros::Time start_time_;         //控制任务的开始时间
  double accumulated_distance_;  //实际累积的控制距离
  double timeout_threshold_;     //超时距离

 private:
  std::string name_;                  //控制器名称
  DependencyInjector::Ptr injector_;  //全局变量
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