#pragma once
#include <cmath>

struct Point {
  double x, y;
};

/**
 * A simple cubic Bezier class with 4 control points (p0..p3).
 * Allows evaluating the position, derivatives, and curvature 
 * at parameter u in [0..1].
 */
class CubicBezier {
public:
  CubicBezier(Point p0, Point p1, Point p2, Point p3);

  // Position at parameter u in [0..1]
  Point getPoint(double u) const;

  // First derivative wrt u
  Point getFirstDerivative(double u) const;

  // Second derivative wrt u
  Point getSecondDerivative(double u) const;

  // Curvature = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
  double getCurvature(double u) const;

private:
  Point p0_, p1_, p2_, p3_;
};
