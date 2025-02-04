#include "ramseteController/ramsete_controller.hpp"
#include <cmath>

RamseteController::RamseteController(double b, double zeta)
  : b_(b), zeta_(zeta)
{
}

std::pair<double,double> RamseteController::calculate(
    double xRobot, double yRobot, double thetaRobot,
    double xRef,   double yRef,   double thetaRef,
    double vRef,   double wRef
) const
{
  // 1) Pose error in global frame
  double dx = xRef - xRobot;
  double dy = yRef - yRobot;
  thetaRef = angleWrap(thetaRef);
  double dTheta = angleWrap(thetaRef - thetaRobot);

  // 2) Transform error into robot frame
  double cosR = std::cos(thetaRobot);
  double sinR = std::sin(thetaRobot);
  double eX =  cosR*dx + sinR*dy;
  double eY = -sinR*dx + cosR*dy;

  // 3) The standard RAMSETE formula
  // k = 2 zeta sqrt(wRef^2 + b vRef^2)
  double k = 2.0 * zeta_ * std::sqrt(wRef*wRef + b_ * vRef*vRef);

  double eTheta = dTheta; // in robot frame, it's the same because we subtracted above

  // Command velocities
  double vCmd = vRef * std::cos(eTheta) + k * eX;
  double wCmd = wRef + k * eY + b_ * vRef * sinc(eTheta) * eTheta;

  return {vCmd, wCmd};
}
