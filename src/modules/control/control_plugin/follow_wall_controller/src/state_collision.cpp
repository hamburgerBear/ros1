#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

void StateCollision::OnEnter(/*后退距离*/) {
  ROS_INFO("StateCollision::OnEnter");
  //判断是否需要后退，软碰撞无需后退
  {
    backward_ = std::make_unique<Translation>("backward", Owner().injector_);
    std::vector<Eigen::Vector3d> path;
    path.emplace_back(-0.02, 0.0, 0.0);
    std::shared_ptr<Translation::TranslationArgs> args =
        std::make_shared<Translation::TranslationArgs>(path, 0.1);
    backward_->setGoal(args);
  }

  {
    rotation_ = std::make_unique<Rotation>("rotation", Owner().injector_);
    std::vector<Eigen::Vector3d> path;
    double theta = calcRatationTheta();
    path.emplace_back(theta, 0.0, 0.0);
    std::shared_ptr<Rotation::RotationArgs> args =
        std::make_shared<Rotation::RotationArgs>(path, 1.0);
    rotation_->setGoal(args);
  }

  which_control_ = "backward";
}

//注意碰撞后的旋转角度也非常关键
//另一个是在右沿墙的情况下，左侧碰撞，是否需要右转，实现穿椅子或者脱困功能。
void StateCollision::Update() {
  ROS_INFO("StateCollision::Update");
  if (which_control_ == "backward") {
    backward_->update();
    if (backward_->isFinish() || backward_->isFail()) {
      //可以重新计算合适的旋转角度
      which_control_ = "rotation";
    }
  } else
    rotation_->update();

  ROS_INFO("which_control(%s)", which_control_.c_str());
}

void StateCollision::OnExit() { ROS_INFO("StateCollision::OnExit"); }

Transition StateCollision::GetTransition() {
  if (which_control_ == "rotation" && rotation_->isFinish() ||
      rotation_->isFail()) {
    if (Owner().loseFollowObject(50.0, 85.0))
      return SiblingTransition<StateApproachWall>();
    else
      return SiblingTransition<StateFollowWall>();
  } else
    return NoTransition();
}

double StateCollision::calcRatationTheta() {
  if (Owner().getFollowDir() == -1.0) {
    if (Owner().injector()->bumper_->data[2]) {
      return toRad(80.0);
    } else if (Owner().injector()->bumper_->data[0] ||
               Owner().injector()->bumper_->data[1]) {
      return toRad(80.0);
    } else if (Owner().injector()->bumper_->data[2] ||
               Owner().injector()->bumper_->data[3]) {
      return toRad(20.0);
    } else {
      ROS_ERROR("is no possible.");
      return toRad(20.0);
    }
  } else {
  }
}

}  // namespace control
