#include "spline_planner/cubic_polynomial.h"

namespace spline_planner {

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
    const std::array<double, 3>& start, const double end, const double param)
    : CubicPolynomialCurve1d(start[0], start[1], start[2], end, param) {}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(const double x0,
                                               const double dx0,
                                               const double ddx0,
                                               const double x1,
                                               const double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, param);
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_ = x1;
}

// void CubicPolynomialCurve1d::DerivedFromQuarticCurve(
//     const PolynomialCurve1d& other) {
//   CHECK_EQ(other.Order(), 4U);
//   param_ = other.ParamLength();
//   for (size_t i = 1; i < 5; ++i) {
//     coef_[i - 1] = other.Coef(i) * static_cast<double>(i);
//   }
// }

double CubicPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                        const double p) const {
  switch (order) {
    case 0: {
      return ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
    }
    case 1: {
      return (3.0 * coef_[3] * p + 2.0 * coef_[2]) * p + coef_[1];
    }
    case 2: {
      return 6.0 * coef_[3] * p + 2.0 * coef_[2];
    }
    case 3: {
      return 6.0 * coef_[3];
    }
    default:
      return 0.0;
  }
}

// std::string CubicPolynomialCurve1d::ToString() const {
//   return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
// }

void CubicPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                 const double dx0,
                                                 const double ddx0,
                                                 const double x1,
                                                 const double param) {
  // DCHECK(param > 0.0);
  const double p2 = param * param;
  const double p3 = param * p2;
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;
  coef_[3] = (x1 - x0 - dx0 * param - coef_[2] * p2) / p3;
}

double CubicPolynomialCurve1d::Coef(const size_t order) const {
  // CHECK_GT(4U, order);
  return coef_[order];
}

CubicPolynomial::CubicPolynomial() {}

CubicPolynomial::~CubicPolynomial() {}

std::vector<geometry_msgs::PoseStamped> CubicPolynomial::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 2) return std::move(out);

  geometry_msgs::PoseStamped start = in[0], goal = in[1];
  /*注意!
  三次多项式轨迹采样时间间隔。不同于三次样条，时间长度是将所有路径点的欧式距离值累加，所以采样分辨率近似于时间间隔。
  而Dubins和RS曲线的轨迹长度和轨迹采样被被封装在类成员函数中，无需外部考虑。
  - 三次多项式的起点方向取决于起点的速度
  - 三次样条根据时间间隔采样，采样点不均匀
  - 三次样条不考虑运动学旋转半径约束*/
  double dx0 = 0.2, ddx0 = 0.2, time_length = 8.0, time_resolution = 0.1;
  double dy0 = 0.5, ddy0 = 0.2;
  CubicPolynomialCurve1d polynomial_x(start.pose.position.x, dx0, ddx0, goal.pose.position.x, time_length);
  CubicPolynomialCurve1d polynomial_y(start.pose.position.y, dx0, ddx0, goal.pose.position.y, time_length);

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










