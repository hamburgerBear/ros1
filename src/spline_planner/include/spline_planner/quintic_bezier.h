#ifndef QUINTIC_BEZIER_H
#define QUINTIC_BEZIER_H

#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>

namespace spline_planner {

// Coefficients for matrix calculation of the quintic Bézier curve.
const Eigen::Matrix<double, 6, 6> quintic_bezier_coefficients(
  (Eigen::Matrix<double, 6, 6>() << 1, 0, 0, 0, 0, 0, -5, 5, 0, 0, 0, 0, 10, -20, 10, 0, 0, 0, -10,
   30, -30, 10, 0, 0, 5, -20, 30, -20, 5, 0, -1, 5, -10, 10, -5, 1)
    .finished());
const Eigen::Matrix<double, 5, 6> quintic_bezier_velocity_coefficients(
  (Eigen::Matrix<double, 5, 6>() << quintic_bezier_coefficients.row(1) * 1,
   quintic_bezier_coefficients.row(2) * 2, quintic_bezier_coefficients.row(3) * 3,
   quintic_bezier_coefficients.row(4) * 4, quintic_bezier_coefficients.row(5) * 5)
    .finished());
const Eigen::Matrix<double, 4, 6> quintic_bezier_acceleration_coefficients(
  (Eigen::Matrix<double, 4, 6>() << quintic_bezier_velocity_coefficients.row(1) * 1,
   quintic_bezier_velocity_coefficients.row(2) * 2, quintic_bezier_velocity_coefficients.row(3) * 3,
   quintic_bezier_velocity_coefficients.row(4) * 4)
    .finished());

/// @brief Quintic Bezier curve
class QuinticBezier
{
  Eigen::Matrix<double, 6, 2> control_points_;

public:
  explicit QuinticBezier();
  /// @brief constructor from a matrix
  explicit QuinticBezier(Eigen::Matrix<double, 6, 2> control_points);
  /// @brief constructor from a set of control points
  explicit QuinticBezier(const std::vector<Eigen::Vector2d> & control_points);
  /// @brief return the control points
  const Eigen::Matrix<double, 6, 2> & getControlPoints() const;
  /// @brief return the curve in cartesian frame with the desired resolution
  std::vector<Eigen::Vector2d> cartesian(const double resolution) const;
  /// @brief return the curve in cartesian frame with the desired number of points
  std::vector<Eigen::Vector2d> cartesian(const int nb_points) const;
  /// @brief return the curve in cartesian frame (including angle) with the desired number of
  /// points
  std::vector<Eigen::Vector3d> cartesianWithHeading(const int nb_points) const;
  /// @brief calculate the curve value for the given parameter t
  Eigen::Vector2d value(const double t) const;
  /// @brief calculate the curve value for the given parameter t (using matrix formulation)
  Eigen::Vector2d valueM(const double t) const;
  /// @brief calculate the velocity (1st derivative) for the given parameter t
  Eigen::Vector2d velocity(const double t) const;
  /// @brief calculate the acceleration (2nd derivative) for the given parameter t
  Eigen::Vector2d acceleration(const double t) const;
  /// @brief return the heading (in radians) of the tangent for the given parameter t
  double heading(const double t) const;
  /// @brief calculate the curvature for the given parameter t
  double curvature(const double t) const;

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);
};

}  // namespace spline_planner

#endif
