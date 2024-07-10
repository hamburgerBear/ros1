#include <spline_planner/quintic_bezier.h>

#include <iostream>

namespace spline_planner
{

QuinticBezier::QuinticBezier() {}

QuinticBezier::QuinticBezier(Eigen::Matrix<double, 6, 2> control_points)
: control_points_(std::move(control_points))
{
}
QuinticBezier::QuinticBezier(const std::vector<Eigen::Vector2d> & control_points)
{
  if (control_points.size() != 6) {
    std::cerr << "Trying to initialize a quintic bezier curve with " << control_points.size()
              << " (!= 6) control points." << std::endl;
  } else {
    control_points_ << control_points[0], control_points[1], control_points[2], control_points[3],
      control_points[4], control_points[5];
  }
}

const Eigen::Matrix<double, 6, 2> & QuinticBezier::getControlPoints() const
{
  return control_points_;
}

Eigen::Vector2d QuinticBezier::value(const double t) const
{
  Eigen::Vector2d point = {0.0, 0.0};
  // sum( binomial(i in 5) * (1 - t)^(5-i) * t^i * control_points_[i] )
  point += std::pow((1 - t), 5) * control_points_.row(0);
  point += 5 * std::pow((1 - t), 4) * t * control_points_.row(1);
  point += 10 * std::pow((1 - t), 3) * t * t * control_points_.row(2);
  point += 10 * std::pow((1 - t), 2) * t * t * t * control_points_.row(3);
  point += 5 * (1 - t) * t * t * t * t * control_points_.row(4);
  point += t * t * t * t * t * control_points_.row(5);
  return point;
}

Eigen::Vector2d QuinticBezier::valueM(const double t) const
{
  Eigen::Matrix<double, 1, 6> ts;
  ts << 1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t;
  return ts * quintic_bezier_coefficients * control_points_;
}

std::vector<Eigen::Vector2d> QuinticBezier::cartesian(const int nb_points) const
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(nb_points);
  const double step = 1.0 / (nb_points - 1);
  for (double t = 0.0; t <= 1.0; t += step) points.push_back(valueM(t));
  return points;
}

std::vector<Eigen::Vector2d> QuinticBezier::cartesian(const double resolution) const
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(static_cast<int>(1 / resolution));
  for (double t = 0.0; t <= 1.0; t += resolution) points.push_back(valueM(t));
  return points;
}

std::vector<Eigen::Vector3d> QuinticBezier::cartesianWithHeading(const int nb_points) const
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(nb_points);
  const double step = 1.0 / (nb_points - 1);
  for (double t = 0.0; t <= 1.0; t += step) {
    Eigen::Vector2d point = valueM(t);
    points.emplace_back(point.x(), point.y(), heading(t));
  }
  return points;
}

Eigen::Vector2d QuinticBezier::velocity(const double t) const
{
  Eigen::Matrix<double, 1, 5> ts;
  ts << 1, t, t * t, t * t * t, t * t * t * t;
  return ts * quintic_bezier_velocity_coefficients * control_points_;
}

Eigen::Vector2d QuinticBezier::acceleration(const double t) const
{
  Eigen::Matrix<double, 1, 4> ts;
  ts << 1, t, t * t, t * t * t;
  return ts * quintic_bezier_acceleration_coefficients * control_points_;
}

double QuinticBezier::curvature(const double t) const
{
  const Eigen::Vector2d vel = velocity(t);
  const Eigen::Vector2d accel = acceleration(t);
  double curvature = std::numeric_limits<double>::infinity();
  const double denominator = std::pow(vel.x() * vel.x() + vel.y() * vel.y(), 3.0 / 2.0);
  if (denominator != 0) curvature = (vel.x() * accel.y() - accel.x() * vel.y()) / denominator;
  return curvature;
}

double QuinticBezier::heading(const double t) const
{
  const Eigen::Vector2d vel = velocity(t);
  return std::atan2(vel.y(), vel.x());
}

std::vector<geometry_msgs::PoseStamped> QuinticBezier::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 6) {
    std::cerr << "Trying to initialize a quintic bezier curve with " << in.size()
              << " (!= 6) control points." << std::endl;
  } else {
    control_points_ << in[0].pose.position.x, in[0].pose.position.y,
                       in[1].pose.position.x, in[1].pose.position.y, 
                       in[2].pose.position.x, in[2].pose.position.y, 
                       in[3].pose.position.x, in[3].pose.position.y, 
                       in[4].pose.position.x, in[4].pose.position.y, 
                       in[5].pose.position.x, in[5].pose.position.y;
  }

  geometry_msgs::PoseStamped goal = in.back();
  
  double d1 = hypot(in[1].pose.position.x - in[0].pose.position.x, 
                    in[1].pose.position.y - in[0].pose.position.y);

  double d2 = hypot(in[2].pose.position.x - in[1].pose.position.x, 
                    in[2].pose.position.y - in[1].pose.position.y);

  double d3 = hypot(in[3].pose.position.x - in[2].pose.position.x, 
                    in[3].pose.position.y - in[2].pose.position.y);

  double d4 = hypot(in[4].pose.position.x - in[3].pose.position.x, 
                    in[4].pose.position.y - in[3].pose.position.y);

  double d5 = hypot(in[5].pose.position.x - in[4].pose.position.x, 
                    in[5].pose.position.y - in[4].pose.position.y);

  double dist = d1 + d2 + d3 + d4 + d5;
  int iterator_num = std::floor(dist / resolution);
  std::vector<double> time(1, 0.0f);
  for(int i = 0; i < iterator_num; i++) 
      time.emplace_back(1.0 / (iterator_num + 1) * (i + 1)) ;
  
  time.emplace_back(1.0f);
  
  geometry_msgs::PoseStamped pose = goal;
  Eigen::Vector2d pose_eigen;
  for(auto t : time) {
    pose_eigen = value(t);
    pose.pose.position.x = pose_eigen.x();
    pose.pose.position.y = pose_eigen.y();
    out.push_back(pose);
  }

  return std::move(out);
}

}  // namespace spline_planner
