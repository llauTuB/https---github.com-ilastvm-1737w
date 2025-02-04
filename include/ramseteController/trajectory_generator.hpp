#pragma once
#include "cubic_bezier.hpp"
#include <vector>

/**
 * Each sample along the path:
 *  s     = arc length so far
 *  x,y   = coordinates at that sample
 *  kappa = curvature
 *  v     = planned linear velocity at that point
 */
struct PathSample {
  double s;
  double x;
  double y;
  double kappa;
  double v;
  double a;
  double u;
};

/**
 * After we do a velocity profile, we can convert to a time-based trajectory:
 *  t       = time from start
 *  x, y    = position
 *  heading = approximate heading
 *  v       = linear speed
 *  w       = angular speed
 */
struct TrajectorySample {
  double t;
  double x;
  double y;
  double heading;
  double v;
  double w;
};



struct AdvancedConstraints {
    double maxVel     = 76.5375;;   // in/s
    double maxAccel   = 125.192;   // in/s^2
    double maxDecel   = 125.192;
    double maxJerk    = 300.0;  // in/s^3 (example)
    double latAccelMax= 100.0;   // lateral accel limit
    double startVel   = 0.0;
    double endVel     = 2.0;
    bool reverse = false;
};


static double limitAccelByJerk(double aPrev, double vPrev, double ds, double maxJerk, bool forward);



/**
 * Generates a velocity profile along the cubic Bezier that respects:
 *  - maxVel
 *  - maxAccel / maxDecel
 *  - curvature-based corner speed (latAccelMax)
 *  - a given startVel and endVel
 *
 * Returns a vector of PathSamples from s=0 to s=length.
 */
std::vector<PathSample> generateVelocityProfile(
      const CubicBezier &bz,  
      const AdvancedConstraints &c, 
      int steps
);

/**
 * Converts a set of PathSamples {s, x, y, kappa, v}
 * into time-based samples {t, x, y, heading, v, w}.
 *
 * The time step between samples i-1 and i is computed as 
 *    dt = (s_i - s_{i-1}) / avgVelocity
 * heading is computed from derivative approach
 * w = kappa * v  if you want rotational speed
 */
std::vector<TrajectorySample> convertToTime(const CubicBezier &bz, const std::vector<PathSample> &samples);
