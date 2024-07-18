#ifndef CUBIC_POLYNOMIAL_H
#define CUBIC_POLYNOMIAL_H

#include <array>

#include <geometry_msgs/PoseStamped.h>

namespace spline_planner {

class CubicPolynomialCurve1d {
 public:
  CubicPolynomialCurve1d() = default;
  virtual ~CubicPolynomialCurve1d() = default;

  CubicPolynomialCurve1d(const std::array<double, 3>& start, const double end,
                         const double param);

  /**
   * x0 is the value when f(x = 0);
   * dx0 is the value when f'(x = 0);
   * ddx0 is the value when f''(x = 0);
   * f(x = param) = x1
   */
  CubicPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                         const double x1, const double param);

  // void DerivedFromQuarticCurve(const PolynomialCurve1d& other);

  double Evaluate(const std::uint32_t order, const double p) const;

  double ParamLength() const { return param_; }
  // std::string ToString() const;

  double Coef(const size_t order) const;

  size_t Order() const { return 3; }

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double param);
  std::array<double, 4> coef_ = {{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  double end_condition_ = 0.0;

  double param_;
};

class CubicPolynomial {
 public:
  explicit CubicPolynomial();
  ~CubicPolynomial();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);
};

}  // namespace spline_planner

#endif
