#pragma once

#include <ros/ros.h>

namespace control {

class Params {
 public:
  struct Common {
    bool use_visualization;
    bool use_simulation;
    double frequency;
    double robot_radius;
    bool use_sensor_bumper;
    bool use_through_gap;
  } common;

  struct Sensor {
    bool use_laser_frontleft;
    bool use_laser_frontright;
    bool use_laser_right;
    double laser_frontleft_height_min;
    double laser_frontleft_height_max;
    double laser_frontright_height_min;
    double laser_frontright_height_max;
    double laser_right_height_min;
    double laser_right_height_max;
  } sensor;

  struct Fit {
    int fit_laser_right_num;
    double fit_line_length_limit;
    double fit_line_gap_limit;
    int residual_laser_right_num;
  } fit;

  struct FollowWall {
    double follow_wall_distance;
    double follow_wall_tolerance;
    double follow_wall_kp1;
    double follow_wall_kp2;
    double follow_wall_kp3;
    double follow_wall_kp4;
    double follow_wall_scale1;
    double follow_wall_scale2;
    double follow_wall_scale3;
    double follow_wall_scale4;
    double linear_vel_max;
    double linear_vel_min;
    double angular_vel_max;
    double linear_acc;
    double linear_dcc;
  } follow;

  struct ArcToWall {
    double radius;
    double linear_vel_max;
    double linear_vel_min;
    double angular_vel_max;
    double linear_acc;
    double linear_dcc;
    double timeout_limit;
  } arc;

  struct RotationToWall {
    double angular_vel_max;
    double angular_vel_min;
    double angular_acc;
    double timeout_limit;
  } rotation;

  struct StraightLine {
    double forward_linear_vel_max;
    double forward_linear_vel_min;
    double forward_linear_acc;
    double forward_linear_dcc;
    double forward_timeout;
    double backward_linear_vel_max;
    double backward_linear_vel_min;
    double backward_linear_acc;
    double backward_linear_dcc;
    double backward_timeout;
  } straight;

  struct FollowBoundary {
    double continue_at_sample_zone_interval;
    double continue_at_sample_zone_distance;
    double trajectory_inflat_radius;
    double forbidden_inflat_radius;
  } boundary;

  explicit Params(ros::NodeHandle& nh);
  ~Params() = default;

  void loadParams();
  std::string toString();

  bool UseVisualization() const { return common.use_visualization; }
  bool UseSimulattion() const { return common.use_simulation; }
  double Frequency() const { return common.frequency; }
  double RobotRadius() const { return common.robot_radius; }
  bool UseSensorBumper() const { return common.use_sensor_bumper; }
  bool UseThroughGap() const { return common.use_through_gap; }
  bool UseLaserFrontLeft() const { return sensor.use_laser_frontleft; }
  bool UseLaserFrontRight() const { return sensor.use_laser_frontright; }
  bool UseLaserRight() const { return sensor.use_laser_right; }
  double LaserFrontLeftHeightMin() const {
    return sensor.laser_frontleft_height_min;
  }
  double LaserFrontLeftHeightMax() const {
    return sensor.laser_frontleft_height_max;
  }
  double LaserFrontRightHeightMin() const {
    return sensor.laser_frontright_height_min;
  }
  double LaserFrontRightHeightMax() const {
    return sensor.laser_frontright_height_max;
  }
  double LaserRightHeightMin() const { return sensor.laser_right_height_min; }
  double LaserRightHeightMax() const { return sensor.laser_right_height_max; }
  int FitLaserRightNum() const { return fit.fit_laser_right_num; }
  double FitLineLengthLimit() const { return fit.fit_line_length_limit; }
  double FitLineGapLimit() const { return fit.fit_line_gap_limit; }
  int ResidualLaserRightNum() const { return fit.residual_laser_right_num; }
  double FollowWallDistance() const { return follow.follow_wall_distance; }
  double FollowWallTolerance() const { return follow.follow_wall_tolerance; }
  double FollowWallKp1() const { return follow.follow_wall_kp1; }
  double FollowWallKp2() const { return follow.follow_wall_kp2; }
  double FollowWallKp3() const { return follow.follow_wall_kp3; }
  double FollowWallKp4() const { return follow.follow_wall_kp4; }
  double FollowWallScale1() const { return follow.follow_wall_scale1; }
  double FollowWallScale2() const { return follow.follow_wall_scale2; }
  double FollowWallScale3() const { return follow.follow_wall_scale3; }
  double FollowWallScale4() const { return follow.follow_wall_scale4; }
  double FollowWallLinearVelMax() const { return follow.linear_vel_max; }
  double FollowWallLinearVelMin() const { return follow.linear_vel_min; }
  double FollowWallAngularMax() const { return follow.angular_vel_max; }
  double FollowWallLinearAcc() const { return follow.linear_acc; }
  double FollowWallLinearDcc() const { return follow.linear_dcc; }
  double ArcRadius() const { return arc.radius; }
  double ArcLinearVelMax() const { return arc.linear_vel_max; }
  double ArcLinearVelMin() const { return arc.linear_vel_min; }
  double ArcAngularMax() const { return arc.angular_vel_max; }
  double ArcLinearAcc() const { return arc.linear_acc; }
  double ArcLinearDcc() const { return arc.linear_dcc; }
  double ArcTimeout() const { return arc.timeout_limit; }
  double RotationAngularVelMax() const { return rotation.angular_vel_max; }
  double RotationAngularVelMin() const { return rotation.angular_vel_min; }
  double RotationAngularAcc() const { return rotation.angular_acc; }
  double RotationTimeout() const { return rotation.timeout_limit; }
  double ForwardLinearVelMax() const { return straight.forward_linear_vel_max; }
  double ForwardLinearVelMin() const { return straight.forward_linear_vel_min; }
  double ForwardLinearAcc() const { return straight.forward_linear_acc; }
  double ForwardLinearDcc() const { return straight.forward_linear_dcc; }
  double ForwardTimeout() const { return straight.forward_timeout; }
  double BackwardLinearVelMax() const {
    return straight.backward_linear_vel_max;
  }
  double BackwardLinearVelMin() const {
    return straight.backward_linear_vel_min;
  }
  double BackwardLinearAcc() const { return straight.backward_linear_acc; }
  double BackwardLinearDcc() const { return straight.backward_linear_dcc; }
  double BackwardTimeout() const { return straight.backward_timeout; }
  double BoundaryTriggerInterval() const {
    return boundary.continue_at_sample_zone_interval;
  }
  double BoundaryTriggerDistance() const {
    return boundary.continue_at_sample_zone_distance;
  }
  double BoundaryInflatRadius() const {
    return boundary.trajectory_inflat_radius;
  }
  double ForbiddenInflatRadius() const {
    return boundary.forbidden_inflat_radius;
  }

  ros::NodeHandle nh_;
};

}  // namespace control
