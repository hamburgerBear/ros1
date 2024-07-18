#include "spline_planner/quartic_polynomial.h"

namespace spline_planner {

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 2>& end,
    const double param)
    : QuarticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                               param) {}

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const double x0, const double dx0, const double ddx0, const double dx1,
    const double ddx1, const double param) {
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_[0] = dx1;
  end_condition_[1] = ddx1;
  ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, param);
}

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const QuarticPolynomialCurve1d& other) {
  param_ = other.param_;
  coef_ = other.coef_;
}

double QuarticPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                          const double p) const {
  switch (order) {
    case 0: {
      return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
             coef_[0];
    }
    case 1: {
      return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
             coef_[1];
    }
    case 2: {
      return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
    }
    case 3: {
      return 24.0 * coef_[4] * p + 6.0 * coef_[3];
    }
    case 4: {
      return 24.0 * coef_[4];
    }
    default:
      return 0.0;
  }
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::FitWithEndPointFirstOrder(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double p) {
  // CHECK_GT(p, 0.0);

  param_ = p;

  coef_[0] = x0;

  coef_[1] = dx0;

  coef_[2] = 0.5 * ddx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;

  double b0 = x1 - coef_[0] - coef_[1] * p - coef_[2] * p2;
  double b1 = dx1 - dx0 - ddx0 * p;

  coef_[4] = (b1 * p - 3 * b0) / p4;
  coef_[3] = (4 * b0 - b1 * p) / p3;
  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::FitWithEndPointSecondOrder(
    const double x0, const double dx0, const double x1, const double dx1,
    const double ddx1, const double p) {
  // CHECK_GT(p, 0.0);

  param_ = p;

  coef_[0] = x0;

  coef_[1] = dx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;

  double b0 = x1 - coef_[0] - coef_[1] * p;
  double b1 = dx1 - coef_[1];
  double c1 = b1 * p;
  double c2 = ddx1 * p2;

  coef_[2] = (0.5 * c2 - 3 * c1 + 6 * b0) / p2;
  coef_[3] = (-c2 + 5 * c1 - 8 * b0) / p3;
  coef_[4] = (0.5 * c2 - 2 * c1 + 3 * b0) / p4;

  return *this;
}

// QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::IntegratedFromCubicCurve(
//     const PolynomialCurve1d& other, const double init_value) {
//   CHECK_EQ(other.Order(), 3U);
//   param_ = other.ParamLength();
//   coef_[0] = init_value;
//   for (size_t i = 0; i < 4; ++i) {
//     coef_[i + 1] = other.Coef(i) / (static_cast<double>(i) + 1);
//   }
//   return *this;
// }

// QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::DerivedFromQuinticCurve(
//     const PolynomialCurve1d& other) {
//   CHECK_EQ(other.Order(), 5U);
//   param_ = other.ParamLength();
//   for (size_t i = 1; i < 6; ++i) {
//     coef_[i - 1] = other.Coef(i) * static_cast<double>(i);
//   }
//   return *this;
// }

void QuarticPolynomialCurve1d::ComputeCoefficients( 
    const double x0, const double dx0, const double ddx0, const double dx1,
    const double ddx1, const double p) {
  // CHECK_GT(p, 0.0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;

  double b0 = dx1 - ddx0 * p - dx0;
  double b1 = ddx1 - ddx0;

  double p2 = p * p;
  double p3 = p2 * p;

  coef_[3] = (3 * b0 - b1 * p) / (3 * p2);
  coef_[4] = (-2 * b0 + b1 * p) / (4 * p3);
}

// std::string QuarticPolynomialCurve1d::ToString() const {
//   return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
// }

double QuarticPolynomialCurve1d::Coef(const size_t order) const {
  // CHECK_GT(5U, order);
  return coef_[order];
}

QuarticPolynomial::QuarticPolynomial() {}

QuarticPolynomial::~QuarticPolynomial() {}

std::vector<geometry_msgs::PoseStamped> QuarticPolynomial::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 2) return std::move(out);

  geometry_msgs::PoseStamped start = in[0], goal = in[1];

  double dx0 = 0.2, ddx0 = 0.0, dx1 = 0.0, ddx1 = 0.0, time_length = 8.0, time_resolution = 0.1;
  double dy0 = 0.2, ddy0 = 0.0, dy1 = 0.0, ddy1 = 0.0;

  //TODO:该计算方式有bug,需要使用FitWithEndPointSecondOrder函数计算
  QuarticPolynomialCurve1d polynomial_x(start.pose.position.x, dx0, ddx0, dx1, ddx1, time_length);
  polynomial_x.FitWithEndPointFirstOrder(start.pose.position.x, dx0, ddx0,
  	                                     goal.pose.position.x, dx1, time_length);
  QuarticPolynomialCurve1d polynomial_y(start.pose.position.y, dy0, ddy0, dy1, ddy1, time_length);
  polynomial_y.FitWithEndPointFirstOrder(start.pose.position.y, dy0, ddy0,
  	                                     goal.pose.position.y, dy1, time_length);
  geometry_msgs::PoseStamped pose = goal;
  for (double t = 0.0; t < time_length; t += time_resolution) {
    pose.pose.position.x = polynomial_x.Evaluate(0, t);
    pose.pose.position.y = polynomial_y.Evaluate(0, t);
    out.push_back(pose);
  }
  
  if(hypot(goal.pose.position.x - out.back().pose.position.x, 
        goal.pose.position.y - out.back().pose.position.y) < (resolution * 0.3))
    out.pop_back();

  out.push_back(goal);
  return std::move(out);
}

}  // namespace spline_planner
