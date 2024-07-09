#include "spline_planner/cubic_spline.h"

namespace spline_planner {

CubicSpline1D::CubicSpline1D() {}

CubicSpline1D::CubicSpline1D(const std::vector<double>& xs,
                             const std::vector<double>& ys)
    : xs_(xs), ys_(ys), nx_(xs.size()), a_(ys) {
  for(unsigned int i = 1; i < xs_.size(); i++)
    h_.push_back(xs_[i] - xs_[i - 1]);

  auto matrixA = [=]() -> Eigen::MatrixXd {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx_, nx_);
    A(0, 0) = 1;
    for(int i = 0; i < nx_ - 1; i++) {
      if(i != nx_ - 2) {
        A(i + 1, i + 1) = 2 * (h_[i] + h_[i + 1]);
      }
      A(i + 1, i) = h_[i];
      A(i, i + 1) = h_[i];
    }
    A(0, 1) = 0.0;
    A(nx_ - 1, nx_ - 2) = 0.0;
    A(nx_ - 1, nx_ - 1) = 1.0;
    return A;
  };
  auto matrixB = [=]() -> Eigen::VectorXd {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx_);
    for(int i = 0; i < nx_ - 2; i++) {
      B(i + 1) = 3.0 * (a_[i + 2] - a_[i + 1]) / h_[i + 1] -
                 3.0 * (a_[i + 1] - a_[i]) / h_[i];
    }
    return B;
  };
  Eigen::MatrixXd A = matrixA();
  Eigen::VectorXd B = matrixB();
  Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
  double* c_pointer = c_eigen.data();
  c_.assign(c_pointer, c_pointer + c_eigen.rows());

  for(int i = 0; i < nx_ - 1; i++) {
    d_.push_back((c_[i + 1] - c_[i]) / (3.0 * h_[i]));
    b_.push_back((a_[i + 1] - a_[i]) / h_[i] -
                 h_[i] * (c_[i + 1] + 2 * c_[i]) / 3.0);
  }
}

double CubicSpline1D::calc(double t) {
  int seg_id = bisect(t, 0, nx_);
  double dx = t - xs_[seg_id];
  return a_[seg_id] + b_[seg_id] * dx + c_[seg_id] * dx * dx +
         d_[seg_id] * dx * dx * dx;
}

double CubicSpline1D::calcFirstDerivative(double t) {
  int seg_id = bisect(t, 0, nx_);
  double dx = t - xs_[seg_id];
  return a_[seg_id] + b_[seg_id] * dx + c_[seg_id] * dx * dx +
         d_[seg_id] * dx * dx * dx;
}

double CubicSpline1D::calcSecondDerivative(double t) {
  int seg_id = bisect(t, 0, nx_ - 1);
  double dx = t - xs_[seg_id];
  return b_[seg_id] + 2 * c_[seg_id] * dx + 3 * d_[seg_id] * dx * dx;
}
int CubicSpline1D::bisect(double t, int start, int end) {
  int mid = (start + end) / 2;
  if(t == xs_[mid] || end - start <= 1) {
    return mid;
  } else if(t > xs_[mid]) {
    return bisect(t, mid, end);
  } else {
    return bisect(t, start, mid);
  }
}

CubicSpline::CubicSpline() {}

CubicSpline::~CubicSpline() {}

std::vector<geometry_msgs::PoseStamped> CubicSpline::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() < 2) return std::move(out);

  std::vector<double> xs, ys;
  for (auto pose : in) {
    xs.push_back(pose.pose.position.x);
    ys.push_back(pose.pose.position.y);
  }

  calcTrajectoryLength(xs, ys, trajectory_length_);
  spline_x_ = CubicSpline1D(trajectory_length_, xs);
  spline_y_ = CubicSpline1D(trajectory_length_, ys);
  max_trajectory_length_ =
      *std::max_element(trajectory_length_.begin(), trajectory_length_.end());

  geometry_msgs::PoseStamped pose = in.back();
  for (double t = 0.0; t < trajectory_length_.back(); t += resolution) {
    if(t > max_trajectory_length_) continue;
    calcPosition(t, pose);
    calcCurvature(t);
    out.push_back(pose);
  }

  if(hypot(in.back().pose.position.x - out.back().pose.position.x, 
        in.back().pose.position.y - out.back().pose.position.y) < (resolution * 0.3))
    out.pop_back();

  out.push_back(in.back());
  return std::move(out);
}

void CubicSpline::calcTrajectoryLength(
    const std::vector<double>& xs, const std::vector<double>& ys,
    std::vector<double>& trajectory_length) {
  trajectory_length.clear();
  double length = 0.0;
  trajectory_length.push_back(length);
  for(unsigned int i = 1; i < xs.size(); i++) {
    length += hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
    trajectory_length.push_back(length);
  }
}

void CubicSpline::calcPosition(double t, geometry_msgs::PoseStamped& pose) {
  pose.pose.position.x = spline_x_.calc(t);
  pose.pose.position.y = spline_y_.calc(t);
}

void CubicSpline::calcCurvature(double t) {
  double dx = spline_x_.calcFirstDerivative(t);
  double ddx = spline_x_.calcSecondDerivative(t);
  double dy = spline_y_.calcFirstDerivative(t);
  double ddy = spline_y_.calcSecondDerivative(t);
  double k = (ddy * dx - ddx * dy) / sqrt(pow(dx * dx + dy * dy, 3));
}

}  // namespace spline_planner
