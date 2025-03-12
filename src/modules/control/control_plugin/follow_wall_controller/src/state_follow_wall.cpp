#include "follow_wall_controller/state_follow_wall.h"

namespace control {

void StateFollowWall::OnEnter() {}

void StateFollowWall::Update() {
  double dir = -1.0;           // Data()->GetFollowDir();
  double robot_radius = 0.18;  // Data()->Params()->RobotRadius();
  double desired_distance =
      robot_radius + 0.02;           // Data()->Params()->FollowWallDistance();
  double tolerance_distance = 0.06;  // Data()->Params()->FollowWallTolerance();

  double section_1 = Owner().getLateralDistanceFromScan(50.0 * dir, 60.0 * dir);
  double section_2 = Owner().getLateralDistanceFromScan(60.0 * dir, 70.0 * dir);
  double section_3 = Owner().getLateralDistanceFromScan(70.0 * dir, 80.0 * dir);
  double section_4 = Owner().getLateralDistanceFromScan(80.0 * dir, 85.0 * dir);

  double lateral_distance;
  double kp;
  bool lose_wall = false;
  int section = 0;
  auto BelieveSection1 = [=]() -> bool {
    if ((section_1 - section_2) >
            (/*Data()->Params()->FollowWallDistance()*/ 0.03 * 0.5) ||
        (section_1 - section_3) >
            (/*Data()->Params()->FollowWallDistance()*/ 0.03 * 0.5) ||
        (section_1 - section_4) >
            (/*Data()->Params()->FollowWallDistance()*/ 0.03 * 0.5))
      return false;

    return true;
  };
  auto BelieveSection2 = [=]() -> bool {
    if ((section_2 - section_3) >
            (/*Data()->Params()->FollowWallDistance()*/ 0.03 * 0.5) ||
        (section_2 - section_4) >
            (/*Data()->Params()->FollowWallDistance()*/ 0.03 * 0.5))
      return false;

    return true;
  };
  auto BelieveSection3 = [=]() -> bool {
    if ((section_3 - section_4) >
        (/*Data()->Params()->FollowWallDistance()*/ 0.03 * 0.5))
      return false;

    return true;
  };
  if (BelieveSection1() &&
      section_1 < (desired_distance + tolerance_distance)) {
    lateral_distance = section_1;
    kp = 20.0;  // Data()->Params()->FollowWallKp1();
    section = 1;
  } else if (BelieveSection2() &&
             section_2 < desired_distance + tolerance_distance) {
    lateral_distance = section_2;
    kp = 20.0;  // Data()->Params()->FollowWallKp2();
    section = 2;
  } else if (BelieveSection3() &&
             section_3 < desired_distance + tolerance_distance) {
    lateral_distance = section_3;
    kp = 20.0;  // Data()->Params()->FollowWallKp3();
    section = 3;
  } else if (section_4 < desired_distance + tolerance_distance) {
    lateral_distance = section_4;
    kp = 20.0;  // Data()->Params()->FollowWallKp4();
    section = 4;
  } else
    lose_wall = true;

  if (lose_wall) {
    // cmd_vel = Data()->last_cmd_;
    return;
  }

  Owner().injector_->cmd_Vel.angular.z =
      (desired_distance - lateral_distance) * kp * -dir;
  Owner().injector_->cmd_Vel.linear.x =
      0.2;  // Data()->Params()->FollowWallLinearVelMax();
}

void StateFollowWall::OnExit() {}

Transition StateFollowWall::GetTransition() {
  // if (Owner->loseObject())
  //   return SiblingTransition<StateArc>();
  // else
  return NoTransition();
}

}  // namespace control
