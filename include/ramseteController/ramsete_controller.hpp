#pragma once
#include <utility>
#include <cmath>

/**
 * A simple RAMSETE controller class.
 *   b   (beta) ~ 2.0
 *   zeta ~ 0.7 - 1.0
 *
 * Usage:
 *   auto [vOut, wOut] = ramsete.calculate(
 *       xRobot, yRobot, thetaRobot,
 *       xRef,   yRef,   thetaRef,
 *       vRef,   wRef
 *   );
 */
class RamseteController {
public:
  RamseteController(double b, double zeta);

  /**
   * Returns (vCmd, wCmd) as a pair.
   *
   * xRobot, yRobot, thetaRobot = actual robot pose
   * xRef,   yRef,   thetaRef   = reference pose
   * vRef,   wRef               = reference linear & angular velocity
   */
  std::pair<double,double> calculate(
      double xRobot,   double yRobot,   double thetaRobot,
      double xRef,     double yRef,     double thetaRef,
      double vRef,     double wRef
  ) const;

private:
  double b_;
  double zeta_;

  // Helper to wrap angles to (-pi, pi)
  double angleWrap(double angle) const {
    while(angle >  M_PI) angle -= 2.0*M_PI;
    while(angle < -M_PI) angle += 2.0*M_PI;
    return angle;
  }

  // "sinc" function
  double sinc(double x) const {
    if(std::fabs(x)<1e-9) return 1.0;
    return std::sin(x)/x;
  }
};
