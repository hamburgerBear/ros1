#include "spline_planner/arc.h"

namespace spline_planner {

Arc::Arc() {}

Arc::~Arc() {}

std::vector<geometry_msgs::PoseStamped> Arc::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 3) return std::move(out);

  auto sign = [](double x) { return x < 0 ? -1.0 : 1.0; };

  auto normalize_angle = [](double angle) {
    if(angle > M_PI)
      angle = angle - 2 * M_PI;
    else if(angle < -M_PI)
      angle = angle + 2 * M_PI;
    return angle;
  };

  double x1 = in[0].pose.position.x;
  double y1 = in[0].pose.position.y;
  double x2 = in[1].pose.position.x;
  double y2 = in[1].pose.position.y;
  double x3 = in[2].pose.position.x;
  double y3 = in[2].pose.position.y;

  double x1x1 = x1*x1;
  double y1y1 = y1*y1;
  double x2x2 = x2*x2;
  double y2y2 = y2*y2;
  double x3x3 = x3*x3;
  double y3y3 = y3*y3;
 
  double x2y3 = x2*y3;
  double x3y2 = x3*y2;

  double x2_x3 = x2-x3;
  double y2_y3 = y2-y3;

  double x1x1py1y1 = x1x1 + y1y1;
  double x2x2py2y2 = x2x2 + y2y2;
  double x3x3py3y3 = x3x3 + y3y3;

  double A = x1 * y2_y3 - y1 * x2_x3 + x2y3 - x3y2;
  double B = x1x1py1y1 * (-y2_y3) + x2x2py2y2 * (y1-y3) + x3x3py3y3 * (y2-y1);
  double C = x1x1py1y1 * x2_x3 + x2x2py2y2 * (x3 - x1) + x3x3py3y3 * (x1-x2);
  double D = x1x1py1y1 * (x3y2 - x2y3) + x2x2py2y2 * (x1*y3 - x3*y1) + x3x3py3y3 * (x2*y1-x1*y2);

  double a = -B/(2*A);
  double b = -C/(2*A);
  double radius = sqrt((B*B+C*C-4*A*D)/(4*A*A));
  
  double theta1, theta2, theta3, diff, diff1, diff2;
  theta1 = atan2(y1-b, x1-a);
  theta2 = atan2(y2-b, x2-a);
  theta3 = atan2(y3-b, x3-a);
  diff1 = normalize_angle(theta2-theta1); 
  diff2 = normalize_angle(theta3-theta1);   

  if( (sign(diff1) != sign(diff2)) || ((sign(diff1) == sign(diff2)) && (fabs(diff1) > fabs(diff2))) ) 
    diff2 = (sign(diff2) > 0)? diff2 - 2*M_PI : diff2 + 2*M_PI;
  
  double arc_len = 2 * fabs(diff2) * radius;
  int point_num = arc_len / resolution;
  double diff_step = diff2 / point_num;

  geometry_msgs::PoseStamped goal = in.back();
  geometry_msgs::PoseStamped pose = goal;
  for(int i = 0; i < point_num; i++) {
    pose.pose.position.x = a + cos(diff_step*i+theta1) * radius;
    pose.pose.position.y = b + sin(diff_step*i+theta1) * radius;
    out.push_back(pose);
  }

  if(hypot(goal.pose.position.x - out.back().pose.position.x, 
        goal.pose.position.y - out.back().pose.position.y) < (resolution * 0.3))
    out.pop_back();

  out.push_back(goal);
  return std::move(out);
}

}  // namespace spline_planner
