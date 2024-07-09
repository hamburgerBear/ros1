#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

namespace spline_planner {

class CubicSpline1D {
 public:
  CubicSpline1D();
  CubicSpline1D(const std::vector<double>& xs, const std::vector<double>& ys);
  double calc(double t);
  double calcFirstDerivative(double t);
  double calcSecondDerivative(double t);

 private:
  int bisect(double t, int start, int end);

  std::vector<double> xs_;
  std::vector<double> ys_;
  int nx_;
  std::vector<double> h_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

class CubicSpline {
 public:
  explicit CubicSpline();
  ~CubicSpline();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);

 private:
  void calcTrajectoryLength(
      const std::vector<double>& xs, const std::vector<double>& ys,
      std::vector<double>& trajectory_length);
  void calcPosition(double t, geometry_msgs::PoseStamped& pose);
  void calcCurvature(double t);

  CubicSpline1D spline_x_;
  CubicSpline1D spline_y_;
  std::vector<double> trajectory_length_;
  double max_trajectory_length_;
};

}  // namespace spline_planner

#endif
