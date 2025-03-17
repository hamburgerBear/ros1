#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

void StateFollowWall::OnEnter() { ROS_INFO("StateFollowWall::OnEnter"); }

void StateFollowWall::Update() {
  double dir = Owner().getFollowDir();
  double robot_radius = Owner().params()->RobotRadius();
  double follow_wall_distance = Owner().params()->FollowWallDistance();
  double desired_distance = robot_radius + follow_wall_distance;
  double tolerance_distance = Owner().params()->FollowWallTolerance();
  double control_range = desired_distance + tolerance_distance;

  double section_1 = Owner().getLateralDistanceFromScan(50.0 * dir, 60.0 * dir);
  double section_2 = Owner().getLateralDistanceFromScan(60.0 * dir, 70.0 * dir);
  double section_3 = Owner().getLateralDistanceFromScan(70.0 * dir, 80.0 * dir);
  double section_4 = Owner().getLateralDistanceFromScan(80.0 * dir, 85.0 * dir);

  auto believeSection1 = [=]() -> bool {
    if ((section_1 < control_range) &&
        (section_1 < (section_2 + follow_wall_distance * 0.5)) &&
        (section_1 < (section_3 + follow_wall_distance * 0.5)) &&
        (section_1 < (section_4 + follow_wall_distance * 0.5)))
      return true;
    else
      return false;
  };

  auto believeSection2 = [=]() -> bool {
    if ((section_2 < control_range) &&
        (section_2 < (section_3 + follow_wall_distance * 0.5)) &&
        (section_2 < (section_4 + follow_wall_distance * 0.5)))
      return true;
    else
      return false;
  };

  auto believeSection3 = [=]() -> bool {
    if ((section_3 < control_range) &&
        (section_3 < (section_4 + follow_wall_distance * 0.5)))
      return true;
    else
      return false;
  };

  auto believeSection4 = [=]() -> bool {
    if ((section_4 < control_range))
      return true;
    else
      return false;
  };

  double lateral_distance;
  double kp;
  int section = 0;
  if (believeSection1()) {
    lateral_distance = section_1;
    kp = Owner().params()->FollowWallKp1();
    section = 1;
  } else if (believeSection2()) {
    lateral_distance = section_2;
    kp = Owner().params()->FollowWallKp2();
    section = 2;
  } else if (believeSection3()) {
    lateral_distance = section_3;
    kp = Owner().params()->FollowWallKp3();
    section = 3;
  } else if (believeSection4()) {
    lateral_distance = section_4;
    kp = Owner().params()->FollowWallKp4();
    section = 4;
  }

  ROS_INFO("section = %d", section);
  if (section == 0) {
    ROS_INFO("lose follow object");
    auto last_vel = Owner().injector()->lastVel();
    Owner().injector()->velocity(last_vel);
    return;
  }

  // 速度约束，角速度过大时，线速度降低
  // 参考区域越靠近侧方(前瞻距离越小)，速度越低
  // 前方越靠近障碍物，速度越低
  // 加速度限制
  double v = Owner().params()->FollowWallLinearVelMax();
  double w = (desired_distance - lateral_distance) * kp * -dir;
  Owner().injector()->velocity(v, w);
  ROS_INFO("cmd_vel = {%f, %f}", v, w);
}

void StateFollowWall::OnExit() { ROS_INFO("StateFollowWall::OnExit"); }

Transition StateFollowWall::GetTransition() {
  if (Owner().loseFollowObject(50.0, 85.0))
    return SiblingTransition<StateApproachWall>();
  else if (Owner().injector()->collision())
    return SiblingTransition<StateCollision>();
  else
    return NoTransition();
}

}  // namespace control
