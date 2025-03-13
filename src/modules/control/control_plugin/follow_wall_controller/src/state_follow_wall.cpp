#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

void StateFollowWall::OnEnter() { ROS_INFO("StateFollowWall::OnEnter"); }

void StateFollowWall::Update() {
  double dir = -1.0;  // Data()->GetFollowDir();
  double robot_radius = Owner().params()->RobotRadius();
  double desired_distance =
      robot_radius + Owner().params()->FollowWallDistance();
  double tolerance_distance = Owner().params()->FollowWallTolerance();

  double section_1 = Owner().getLateralDistanceFromScan(50.0 * dir, 60.0 * dir);
  double section_2 = Owner().getLateralDistanceFromScan(60.0 * dir, 70.0 * dir);
  double section_3 = Owner().getLateralDistanceFromScan(70.0 * dir, 80.0 * dir);
  double section_4 = Owner().getLateralDistanceFromScan(80.0 * dir, 85.0 * dir);
  ROS_INFO("section_1 = %f", section_1);
  ROS_INFO("section_2 = %f", section_2);
  ROS_INFO("section_3 = %f", section_3);
  ROS_INFO("section_4 = %f", section_4);

  double lateral_distance;
  double kp;
  bool lose_wall = false;
  int section = 0;

  auto BelieveSection1 = [=]() -> bool {
    if ((section_1 - section_2) >
            (Owner().params()->FollowWallDistance() * 0.5) ||
        (section_1 - section_3) >
            (Owner().params()->FollowWallDistance() * 0.5) ||
        (section_1 - section_4) >
            (Owner().params()->FollowWallDistance() * 0.5))
      return false;

    return true;
  };

  auto BelieveSection2 = [=]() -> bool {
    if ((section_2 - section_3) >
            (Owner().params()->FollowWallDistance() * 0.5) ||
        (section_2 - section_4) >
            (Owner().params()->FollowWallDistance() * 0.5))
      return false;

    return true;
  };

  auto BelieveSection3 = [=]() -> bool {
    if ((section_3 - section_4) >
        (Owner().params()->FollowWallDistance() * 0.5))
      return false;

    return true;
  };

  if (BelieveSection1() &&
      section_1 < (desired_distance + tolerance_distance)) {
    lateral_distance = section_1;
    kp = Owner().params()->FollowWallKp1();
    section = 1;
  } else if (BelieveSection2() &&
             section_2 < desired_distance + tolerance_distance) {
    lateral_distance = section_2;
    kp = Owner().params()->FollowWallKp2();
    section = 2;
  } else if (BelieveSection3() &&
             section_3 < desired_distance + tolerance_distance) {
    lateral_distance = section_3;
    kp = Owner().params()->FollowWallKp3();
    section = 3;
  } else if (section_4 < desired_distance + tolerance_distance) {
    lateral_distance = section_4;
    kp = Owner().params()->FollowWallKp4();
    section = 4;
  } else
    lose_wall = true;

  if (lose_wall) {
    // cmd_vel = Data()->last_cmd_;
    return;
  }

  std::cout << "desired_distance = " << desired_distance << std::endl;
  std::cout << "lateral_distance = " << lateral_distance << std::endl;
  std::cout << "kp = " << kp << std::endl;
  // Owner().injector()->cmd_vel_.angular.z = 0.0;
  // Owner().injector()->cmd_vel_.linear.x = 0.0;
  Owner().injector()->cmd_vel_.angular.z =
      (desired_distance - lateral_distance) * kp * -dir;
  Owner().injector()->cmd_vel_.linear.x =
      Owner().params()->FollowWallLinearVelMax();
  ROS_INFO("cmd_vel = {%f, %f}", Owner().injector()->cmd_vel_.linear.x,
           Owner().injector()->cmd_vel_.angular.z);
}

void StateFollowWall::OnExit() { ROS_INFO("StateFollowWall::OnExit"); }

Transition StateFollowWall::GetTransition() {
  if (Owner().loseFollowObject(50.0, 85.0))
    return SiblingTransition<StateApproachWall>();
  else
    return NoTransition();
}

}  // namespace control
