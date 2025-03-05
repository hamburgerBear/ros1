#pragma once

// 这个状态机的基类，可以考虑放在除deliberation、planning、control之外的common中，或是在deliberation的common/base中
namespace control {

class StateBase {
 protected:
  StateBase(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr/*,
    const std::shared_ptr<RouteHandler> & route_handler_ptr*/);
  Status status_;
  // LaneChangerParameters ros_parameters_;
  std::shared_ptr<DataManager> data_manager_ptr_;
  // std::shared_ptr<RouteHandler> route_handler_ptr_;

 public:
  virtual void entry() = 0;
  virtual void update(geometry_msgs::Twist& cmd_vel) = 0;
  virtual State getNextState() const = 0;
  virtual State getCurrentState() const = 0;
  // virtual autoware_planning_msgs::PathWithLaneId getPath() const = 0;

  Status getStatus() const;
};

#include "dependency_injector.h"

enum StateCode {
  FINISH = 0,
  RUNNING = 1,  //状态码
  FAIL = 2      //错误码
}

class StateBase {
 public:
  explicit StateBase(const std::string& name,
                     const DependencyInjector::Ptr& injector) = {

  } virtual ~StateBase() = default;

  virtual StateCode entry() std::string Name() { return name_; }

 private:
  std::string name_;
  DependencyInjector::Ptr injector_;
}

class StateBase {
 public:
  virtual std::string name_;  //状态名字
}

}  // namespace control