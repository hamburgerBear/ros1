#pragma once

#include "follow_wall_controller/follow_wall_cfg.h"

namespace control {

Params::Params(ros::NodeHandle& nh) : nh_(nh) {
  // common
  common.use_visualization = false;
  common.use_simulation = false;
  common.frequency = 20.0f;
  common.robot_radius = 0.18f;
  common.use_sensor_bumper = false;
  common.use_through_gap = false;

  // sensor
  sensor.use_laser_frontleft = false;
  sensor.use_laser_frontright = false;
  sensor.use_laser_right = true;
  sensor.laser_frontleft_height_min = 0.02f;
  sensor.laser_frontleft_height_max = 0.05f;
  sensor.laser_frontright_height_min = 0.02f;
  sensor.laser_frontright_height_max = 0.05f;
  sensor.laser_right_height_min = 0.02f;
  sensor.laser_right_height_max = 0.05f;

  // fit
  fit.fit_laser_right_num = 10;
  fit.fit_line_length_limit = 0.1;
  fit.fit_line_gap_limit = 0.05;
  fit.residual_laser_right_num = 6;

  // follow wall
  follow.follow_wall_distance = 0.03f;
  follow.follow_wall_tolerance = 0.05f;
  follow.follow_wall_kp1 = 10.0f;
  follow.follow_wall_kp2 = 10.0f;
  follow.follow_wall_kp3 = 10.0f;
  follow.follow_wall_kp4 = 10.0f;
  follow.follow_wall_scale1 = 1.0f;
  follow.follow_wall_scale2 = 1.0f;
  follow.follow_wall_scale3 = 1.0f;
  follow.follow_wall_scale4 = 1.0f;
  follow.linear_vel_max = 0.1f;
  follow.linear_vel_min = 0.1f;
  follow.angular_vel_max = 1.0f;
  follow.linear_acc = 10.0f;
  follow.linear_dcc = -10.0f;

  // arc to wall
  arc.radius = 0.15f;
  arc.linear_vel_max = 0.1f;
  arc.linear_vel_min = 0.05f;
  arc.angular_vel_max = 1.0f;
  arc.linear_acc = 10.0f;
  arc.linear_dcc = 10.0f;
  arc.timeout_limit = 5.0f;

  // rotation to wall
  rotation.angular_vel_max = 1.0f;
  rotation.angular_vel_min = 0.05f;
  rotation.angular_acc = 10.0f;
  rotation.timeout_limit = 5.0f;

  // straight line
  straight.forward_linear_vel_max = 0.3f;
  straight.forward_linear_vel_min = 0.05f;
  straight.forward_linear_acc = 10.0f;
  straight.forward_linear_dcc = 10.0f;
  straight.forward_timeout = 5.0f;
  straight.backward_linear_vel_max = 0.3f;
  straight.backward_linear_vel_min = 0.05f;
  straight.backward_linear_acc = 10.0f;
  straight.backward_linear_dcc = 10.0f;
  straight.backward_timeout = 5.0f;

  // follow boundary
  boundary.continue_at_sample_zone_interval = 5.0;
  boundary.continue_at_sample_zone_distance = 0.05;
  boundary.trajectory_inflat_radius = 0.2;
  boundary.forbidden_inflat_radius = 0.15;

  loadParams();
}

void Params::loadParams() {
  // common
  nh_.param("use_visualization", common.use_visualization,
            common.use_visualization);
  nh_.param("use_simulation", common.use_simulation, common.use_simulation);
  nh_.param("frequency", common.frequency, common.frequency);
  nh_.param("robot_radius", common.robot_radius, common.robot_radius);
  nh_.param("use_sensor_bumper", common.use_sensor_bumper,
            common.use_sensor_bumper);
  nh_.param("use_through_gap", common.use_through_gap, common.use_through_gap);

  // sensor
  nh_.param("use_laser_frontleft", sensor.use_laser_frontleft,
            sensor.use_laser_frontleft);
  nh_.param("use_laser_frontright", sensor.use_laser_frontright,
            sensor.use_laser_frontright);
  nh_.param("use_laser_right", sensor.use_laser_right, sensor.use_laser_right);
  nh_.param("laser_frontleft_height_min", sensor.laser_frontleft_height_min,
            sensor.laser_frontleft_height_min);
  nh_.param("laser_frontleft_height_max", sensor.laser_frontleft_height_max,
            sensor.laser_frontleft_height_max);
  nh_.param("laser_frontright_height_min", sensor.laser_frontright_height_min,
            sensor.laser_frontright_height_min);
  nh_.param("laser_frontright_height_max", sensor.laser_frontright_height_max,
            sensor.laser_frontright_height_max);
  nh_.param("laser_right_height_min", sensor.laser_right_height_min,
            sensor.laser_right_height_min);
  nh_.param("laser_right_height_max", sensor.laser_right_height_max,
            sensor.laser_right_height_max);

  // fit
  nh_.param("fit_laser_right_num", fit.fit_laser_right_num,
            fit.fit_laser_right_num);
  nh_.param("fit_line_length_limit", fit.fit_line_length_limit,
            fit.fit_line_length_limit);
  nh_.param("fit_line_gap_limit", fit.fit_line_gap_limit,
            fit.fit_line_gap_limit);
  nh_.param("residual_laser_right_num", fit.residual_laser_right_num,
            fit.residual_laser_right_num);

  // follow wall
  nh_.param("follow_wall_distance", follow.follow_wall_distance,
            follow.follow_wall_distance);
  nh_.param("follow_wall_tolerance", follow.follow_wall_tolerance,
            follow.follow_wall_tolerance);
  nh_.param("follow_wall_kp1", follow.follow_wall_kp1, follow.follow_wall_kp1);
  nh_.param("follow_wall_kp2", follow.follow_wall_kp2, follow.follow_wall_kp2);
  nh_.param("follow_wall_kp3", follow.follow_wall_kp3, follow.follow_wall_kp3);
  nh_.param("follow_wall_kp4", follow.follow_wall_kp4, follow.follow_wall_kp4);
  nh_.param("follow_wall_scale1", follow.follow_wall_scale1,
            follow.follow_wall_scale1);
  nh_.param("follow_wall_scale2", follow.follow_wall_scale2,
            follow.follow_wall_scale2);
  nh_.param("follow_wall_scale3", follow.follow_wall_scale3,
            follow.follow_wall_scale3);
  nh_.param("follow_wall_scale4", follow.follow_wall_scale4,
            follow.follow_wall_scale4);
  nh_.param("follow_wall_linear_vel_max", follow.linear_vel_max,
            follow.linear_vel_max);
  nh_.param("follow_wall_linear_vel_min", follow.linear_vel_min,
            follow.linear_vel_min);
  nh_.param("follow_wall_angular_vel_max", follow.angular_vel_max,
            follow.angular_vel_max);
  nh_.param("follow_wall_linear_acc", follow.linear_acc, follow.linear_acc);
  nh_.param("follow_wall_linear_dcc", follow.linear_dcc, follow.linear_dcc);

  // arc to wall
  nh_.param("arc_radius", arc.radius, arc.radius);
  nh_.param("arc_linear_vel_max", arc.linear_vel_max, arc.linear_vel_max);
  nh_.param("arc_linear_vel_min", arc.linear_vel_min, arc.linear_vel_min);
  nh_.param("arc_angular_vel_max", arc.angular_vel_max, arc.angular_vel_max);
  nh_.param("arc_linear_acc", arc.linear_acc, arc.linear_acc);
  nh_.param("arc_linear_dcc", arc.linear_dcc, arc.linear_dcc);
  nh_.param("arc_timeout_limit", arc.timeout_limit, arc.timeout_limit);

  // rotation to wall
  nh_.param("rotation_angular_vel_max", rotation.angular_vel_max,
            rotation.angular_vel_max);
  nh_.param("rotation_angular_vel_min", rotation.angular_vel_min,
            rotation.angular_vel_min);
  nh_.param("rotation_angular_acc", rotation.angular_acc, rotation.angular_acc);
  nh_.param("rotation_timeout_limit", rotation.timeout_limit,
            rotation.timeout_limit);

  // straight line
  nh_.param("forward_linear_vel_max", straight.forward_linear_vel_max,
            straight.forward_linear_vel_max);
  nh_.param("forward_linear_vel_min", straight.forward_linear_vel_min,
            straight.forward_linear_vel_min);
  nh_.param("forward_linear_acc", straight.forward_linear_acc,
            straight.forward_linear_acc);
  nh_.param("forward_linear_dcc", straight.forward_linear_dcc,
            straight.forward_linear_dcc);
  nh_.param("forward_timeout", straight.forward_timeout,
            straight.forward_timeout);
  nh_.param("backward_linear_vel_max", straight.backward_linear_vel_max,
            straight.backward_linear_vel_max);
  nh_.param("backward_linear_vel_min", straight.backward_linear_vel_min,
            straight.backward_linear_vel_min);
  nh_.param("backward_linear_acc", straight.backward_linear_acc,
            straight.backward_linear_acc);
  nh_.param("backward_linear_dcc", straight.backward_linear_dcc,
            straight.backward_linear_dcc);
  nh_.param("backward_timeout", straight.backward_timeout,
            straight.backward_timeout);

  // follow boundary
  nh_.param("continue_at_sample_zone_interval",
            boundary.continue_at_sample_zone_interval,
            boundary.continue_at_sample_zone_interval);
  nh_.param("continue_at_sample_zone_distance",
            boundary.continue_at_sample_zone_distance,
            boundary.continue_at_sample_zone_distance);
  nh_.param("trajectory_inflat_radius", boundary.trajectory_inflat_radius,
            boundary.trajectory_inflat_radius);
  nh_.param("forbidden_inflat_radius", boundary.forbidden_inflat_radius,
            boundary.forbidden_inflat_radius);

  if (UseVisualization()) std::cout << toString() << std::endl;
}

std::string Params::toString() {
  std::stringstream str;
  str.precision(10);
  str << "[common follower params]" << std::endl;

  str << "(common) "
      << "use_visualization:" << common.use_visualization
      << ", use_simulation:" << common.use_simulation
      << ", frequency:" << common.frequency
      << ", robot_radius:" << common.robot_radius
      << ", use_sensor_bumper:" << common.use_sensor_bumper
      << ", use_through_gap:" << common.use_through_gap << std::endl;

  str << "(sensor) "
      << "use_laser_frontleft:" << sensor.use_laser_frontleft
      << ", use_laser_frontright:" << sensor.use_laser_frontright
      << ", use_laser_right:" << sensor.use_laser_right
      << ", laser_frontleft_height_min:" << sensor.laser_frontleft_height_min
      << ", laser_frontleft_height_max:" << sensor.laser_frontleft_height_max
      << ", laser_frontright_height_min:" << sensor.laser_frontright_height_min
      << ", laser_frontright_height_max:" << sensor.laser_frontright_height_max
      << ", laser_right_height_min:" << sensor.laser_right_height_min
      << ", laser_right_height_max:" << sensor.laser_right_height_max
      << std::endl;

  str << "(fit) "
      << "fit_laser_right_num:" << fit.fit_laser_right_num
      << ", fit_line_length_limit:" << fit.fit_line_length_limit
      << ", fit_line_gap_limit:" << fit.fit_line_gap_limit
      << ", residual_laser_right_num:" << fit.residual_laser_right_num
      << std::endl;

  str << "(follow) "
      << "follow_wall_distance:" << follow.follow_wall_distance
      << ", follow_wall_tolerance:" << follow.follow_wall_tolerance
      << ", follow_wall_kp1:" << follow.follow_wall_kp1
      << ", follow_wall_kp2:" << follow.follow_wall_kp2
      << ", follow_wall_kp3:" << follow.follow_wall_kp3
      << ", follow_wall_kp4:" << follow.follow_wall_kp4
      << ", follow_wall_scale1:" << follow.follow_wall_scale1
      << ", follow_wall_scale2:" << follow.follow_wall_scale2
      << ", follow_wall_scale3:" << follow.follow_wall_scale3
      << ", follow_wall_scale4:" << follow.follow_wall_scale4
      << ", linear_vel_max:" << follow.linear_vel_max
      << ", linear_vel_min:" << follow.linear_vel_min
      << ", angular_vel_max:" << follow.angular_vel_max
      << ", linear_acc:" << follow.linear_acc
      << ", linear_dcc:" << follow.linear_dcc << std::endl;

  str << "(arc) "
      << "radius:" << arc.radius << ", linear_vel_max:" << arc.linear_vel_max
      << ", linear_vel_min:" << arc.linear_vel_min
      << ", angular_vel_max:" << arc.angular_vel_max
      << ", linear_acc:" << arc.linear_acc << ", linear_dcc:" << arc.linear_dcc
      << ", timeout_limit:" << arc.timeout_limit << std::endl;

  str << "(rotation) "
      << "rotation_angular_vel_max:" << rotation.angular_vel_max
      << ", rotation_angular_vel_min:" << rotation.angular_vel_min
      << ", rotation_angular_acc:" << rotation.angular_acc
      << ", rotation_timeout_limit:" << rotation.timeout_limit << std::endl;

  str << "(straight) "
      << "forward_linear_vel_max:" << straight.forward_linear_vel_max
      << ", forward_linear_vel_min:" << straight.forward_linear_vel_min
      << ", forward_linear_acc:" << straight.forward_linear_acc
      << ", forward_linear_dcc:" << straight.forward_linear_dcc
      << ", forward_timeout:" << straight.forward_timeout
      << ", backward_linear_vel_max:" << straight.backward_linear_vel_max
      << ", backward_linear_vel_min:" << straight.backward_linear_vel_min
      << ", backward_linear_acc:" << straight.backward_linear_acc
      << ", backward_linear_dcc:" << straight.backward_linear_dcc
      << ", backward_timeout:" << straight.backward_timeout << std::endl;

  str << "(boundary) "
      << "continue_at_sample_zone_interval:"
      << boundary.continue_at_sample_zone_interval
      << ", continue_at_sample_zone_distance:"
      << boundary.continue_at_sample_zone_distance
      << ", trajectory_inflat_radius:" << boundary.trajectory_inflat_radius
      << ", forbidden_inflat_radius:" << boundary.forbidden_inflat_radius
      << std::endl;

  return str.str();
}

}  // namespace control