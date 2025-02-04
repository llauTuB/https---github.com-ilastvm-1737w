#include "ramseteController/cubic_bezier.hpp"
#include <cmath>

CubicBezier::CubicBezier(Point p0, Point p1, Point p2, Point p3)
  : p0_(p0), p1_(p1), p2_(p2), p3_(p3)
{
}

Point CubicBezier::getPoint(double u) const {
  // B(u) = (1-u)^3 p0 + 3(1-u)^2 u p1 + 3(1-u) u^2 p2 + u^3 p3
  double bu  = 1.0 - u;
  double bu2 = bu * bu;
  double u2  = u * u;

  double x = bu2*bu * p0_.x
           + 3*bu2*u  * p1_.x
           + 3*bu*u2  * p2_.x
           + u2*u     * p3_.x;
  double y = bu2*bu * p0_.y
           + 3*bu2*u  * p1_.y
           + 3*bu*u2  * p2_.y
           + u2*u     * p3_.y;
  return {x, y};
}

Point CubicBezier::getFirstDerivative(double u) const {
  // d/du of cubic Bezier
  double bu = 1.0 - u;
  double x = 3*bu*bu*(p1_.x - p0_.x)
           + 6*bu*u  *(p2_.x - p1_.x)
           + 3*u*u   *(p3_.x - p2_.x);
  double y = 3*bu*bu*(p1_.y - p0_.y)
           + 6*bu*u  *(p2_.y - p1_.y)
           + 3*u*u   *(p3_.y - p2_.y);
  return {x, y};
}

Point CubicBezier::getSecondDerivative(double u) const {
  // second derivative
  // 6(1-u)(p2 - 2p1 + p0) + 6u(p3 - 2p2 + p1)
  double x = 6*(1.0 - u)*(p2_.x - 2*p1_.x + p0_.x)
           + 6*u       *(p3_.x - 2*p2_.x + p1_.x);
  double y = 6*(1.0 - u)*(p2_.y - 2*p1_.y + p0_.y)
           + 6*u       *(p3_.y - 2*p2_.y + p1_.y);
  return {x, y};
}

double CubicBezier::getCurvature(double u) const {
  Point d1 = getFirstDerivative(u);
  Point d2 = getSecondDerivative(u);

  double cross = d1.x*d2.y - d1.y*d2.x; // 2D cross product
  double denom = (d1.x*d1.x + d1.y*d1.y);
  double denomPow = denom * std::sqrt(denom); // denom^(3/2)
  if (denomPow < 1e-12) {
    // derivative is nearly zero => curvature undefined/infinite
    // we'll just return 0 as a fallback
    return std::copysign(0.0, cross);
  }
  return cross / denomPow;
}
