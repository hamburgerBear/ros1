#pragma once

#include <unordered_map>

#include "control_base.h"
#include "dependency_injector.h"

namespace control {

class StateMachine {
 public:
  using Ptr = std::shared_ptr<StateMachine>;

  explicit StateMachine(const DependencyInjector::Ptr& injector)
      : injector_(injector) {}
  virtual ~StateMachine() = default;

  //再注册完后需要初始化状态。
  void registerState(const std::string& name, const ControlBase::Ptr& state) {
    state_list_[name] = state;
  }

  bool initState(const std::string& init_state,
                 std::shared_ptr<ControlBase::Args> args) {
    curr_state_ = init_state;
    // state_list_[curr_state_]->init(args);
    //初始化失败，异常的初始化状态，在状态列表中。
  }

  void proc() {
    //如果需要子状态传递出来，再改接口
    auto state_obj_ptr = state_list_[curr_state_];
    state_obj_ptr->update(/*injector()->cmd_vel*/);
    auto transition = state_obj_ptr->transition();

    if (curr_state_ != transition.first) {
      curr_state_ = transition.first;
      state_obj_ptr = state_list_[curr_state_];
      // state_obj_ptr->init(transition.second);
      state_obj_ptr->update(/*Injector()->cmd_vel*/);
    }
  }

 private:
  DependencyInjector::Ptr injector_;
  std::string curr_state_;
  std::unordered_map<std::string, ControlBase::Ptr> state_list_;
};

// void StateMachine::registerState(const std::string& name,
//                                  const ControlBase::Ptr& state) {
//   state_list_[name] = state;
// }

// bool StateMachine::initState(const std::string& init_state,
//                              std::shared_ptr<ControlBase::Args> args) {
//   curr_state_ = init_state;
//   state_list_[curr_state_]->init(args);
//   //初始化失败，异常的初始化状态，在状态列表中。
// }

// void StateMachine::proc() {
//   //如果需要子状态传递出来，再改接口
//   auto state_obj_ptr = state_list_[curr_state_];
//   state_obj_ptr->update(/*injector()->cmd_vel*/);
//   auto transition = state_obj_ptr->transition();

//   if (curr_state_ != transition.first) {
//     curr_state_ = transition.first;
//     state_obj_ptr = state_list_[curr_state_];
//     state_obj_ptr->init(transition.second);
//     state_obj_ptr->update(/*Injector()->cmd_vel*/);
//   }
// }

}  // namespace control