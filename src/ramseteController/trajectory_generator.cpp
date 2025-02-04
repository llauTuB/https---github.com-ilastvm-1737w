#include "ramseteController/trajectory_generator.hpp"
#include <cmath>
#include <algorithm>

/**
 * Approximate the total arc length of the Bezier by
 * sampling many points and summing distances.
 */
static double approximateLength(const CubicBezier &bz, int steps) {
  Point last = bz.getPoint(0.0);
  double length = 0.0;
  for(int i = 1; i <= steps; i++){
    double u = double(i) / steps;
    Point curr = bz.getPoint(u);
    double dx = curr.x - last.x;
    double dy = curr.y - last.y;
    length += std::sqrt(dx*dx + dy*dy);
    last = curr;
  }
  return length;
}

/**
 * Jerk-limited acceleration function.
 * If 'forward' is true, we try to increase acceleration by (maxJerk * dt),
 * else we decrease by (maxJerk * dt).
 */
static double limitAccelByJerk(double aPrev, double vPrev, double ds, double maxJerk, bool forward) {
    // If velocity is very small, use a small dt to avoid dividing by zero
    double effectiveV = (vPrev < 1e-3) ? 1e-3 : vPrev;
    double dt = ds / effectiveV;

    if (forward) {
        return aPrev + maxJerk * dt;
    } else {
        return aPrev - maxJerk * dt;
    }
}

/** 
 * A helper to do a standard "forward pass":
 *   - We accelerate from samples[i-1] to samples[i] subject to jerk-limited acceleration.
 *   - Then clamp by maxVel, corner speed, etc.
 *   - The partial result is stored back in samples[i].v / .a.
 * 
 * If you want to *also* clamp to a prior pass's velocity, pass 'vPrevPass' in
 * and do   vClamped = std::min(vPossible, vPrevPass).
 * If you do NOT want to clamp to prior pass, pass something big like 1e9.
 */
static void forwardPass(
    std::vector<PathSample> &samples,
    double maxAccel,
    double maxJerk,
    double maxVel,
    double latAccelMax,
    double startVel,
    double vClampFromPrior = 1e9
) {
    // Start velocity for sample[0]
    samples[0].v = std::min(startVel, maxVel);
    samples[0].a = 0.0;

    for(int i = 1; i < (int)samples.size(); i++){
        double ds_i = samples[i].s - samples[i-1].s;
        if(ds_i < 1e-12) {
            samples[i].v = samples[i-1].v;
            samples[i].a = samples[i-1].a;
            continue;
        }

        double vPrev = samples[i-1].v;
        double aPrev = samples[i-1].a;

        // Jerk-limit from aPrev
        double aJerkLimited = limitAccelByJerk(aPrev, vPrev, ds_i, maxJerk, /*forward=*/true);

                // 2) If vPrev is basically zero, forcibly jump-start
        // if (vPrev < 1e-9) {
        //     // forcibly accelerate from 0 with full maxAccel
        //     aJerkLimited = maxAccel;
        // }


        // clamp by +maxAccel
        if(aJerkLimited > maxAccel) aJerkLimited = maxAccel;

        // from a[i], get v(i) = sqrt(vPrev^2 + 2*a[i]*ds_i)
        double vPossible = std::sqrt(std::max(0.0, vPrev*vPrev + 2.0*aJerkLimited*ds_i));

        // corner speed limit
        double vCorner = (samples[i].kappa > 1e-9)
           ? std::sqrt(latAccelMax / samples[i].kappa)
           : maxVel;

        // clamp to that corner limit, global maxVel, and any "previous pass" clamp
        double vClamped = std::min({vPossible, maxVel, vCorner, vClampFromPrior});

        // recalc a
        double newA = 0.0;
        if(ds_i > 1e-9) {
            newA = (vClamped*vClamped - vPrev*vPrev) / (2.0*ds_i);
        }

        samples[i].v = vClamped;
        samples[i].a = newA;
    }
}

/** 
 * A helper to do a standard "backward pass":
 *   - We decelerate from samples[i+1] to samples[i], subject to jerk-limited decel.
 *   - Then clamp by corner speed, etc.
 * 
 * If you want to *also* clamp to a prior pass's velocity, pass 'vPrevPass' in
 * and do   vClamped = std::min(vCurr, vPossible, vPrevPass).
 * If not, pass 1e9.
 */
static void backwardPass(
    std::vector<PathSample> &samples,
    double maxAccel,
    double maxDecel,
    double maxJerk,
    double maxVel,
    double latAccelMax,
    double endVel,
    double vClampFromPrior = 1e9
) {
    // clamp last sample's velocity
    samples.back().v = std::min(samples.back().v, endVel);
    samples.back().a = 0.0;

    for(int i = (int)samples.size() - 2; i >= 0; i--){
        double ds_i = samples[i+1].s - samples[i].s;
        if(ds_i < 1e-12) continue;

        double vNext = samples[i+1].v;
        double aNext = samples[i+1].a;
        double vCurr = samples[i].v;
        double aCurr = samples[i].a;

        // jerk-limit decel from aNext
        double aJerkMin;
        if(vNext < 1e-9) {
            aJerkMin = aNext;
        } else {
            double dt = ds_i / vNext;
            aJerkMin = aNext - maxJerk*dt;  // decel
        }
        // also must not exceed +maxAccel or go below -maxDecel
        double newA = std::min(aCurr, aJerkMin);
        if(newA < -maxDecel) newA = -maxDecel;

        // vPossible = sqrt(vNext^2 - 2* newA * ds_i)
        double vPossible = std::sqrt(std::max(0.0, vNext*vNext - 2.0*newA*ds_i));

        // corner speed
        double vCorner = (samples[i].kappa > 1e-9)
            ? std::sqrt(latAccelMax / samples[i].kappa)
            : maxVel;

        // clamp with vCurr, corner speed, vClampFromPrior, global maxVel
        double vClamped = std::min({vCurr, vPossible, vCorner, maxVel, vClampFromPrior});

        // recalc finalA
        double finalA = 0.0;
        if(ds_i > 1e-9) {
            finalA = (vClamped*vClamped - vNext*vNext) / (-2.0*ds_i);
        }

        samples[i].v = vClamped;
        samples[i].a = finalA;
    }
}


std::vector<PathSample> generateVelocityProfile(
    const CubicBezier &bz,
    const AdvancedConstraints &c,
    int steps /* = 500 */
) {
    // 1) sample the path
    double totalLen = approximateLength(bz, steps);
    if(totalLen < 1e-9) {
        // degenerate path
        std::vector<PathSample> single;
        Point p0 = bz.getPoint(0.0);
        single.push_back({0.0, p0.x, p0.y, 0.0, 0.0, 0.0});
        return single;
    }

    double ds = totalLen / steps;
    std::vector<PathSample> samples;
    samples.reserve(steps+1);

    double sAccum = 0.0;
    Point lastP = bz.getPoint(0.0);

    for(int i = 0; i <= steps; i++){
        double u = double(i) / steps;
        Point p = bz.getPoint(u);

        if(i > 0) {
            double dx = p.x - lastP.x;
            double dy = p.y - lastP.y;
            sAccum += std::sqrt(dx*dx + dy*dy);
        }
        double kappa = std::fabs(bz.getCurvature(u));
        // init velocity to maxVel, accel=0
        samples.push_back({sAccum, p.x, p.y, kappa, c.maxVel, 0.0, u});
        lastP = p;
    }
    samples.back().s = totalLen;

    //-----------------------------------------
    // PASS 1: Forward pass from startVel
    //-----------------------------------------
    forwardPass(samples,
                c.maxAccel, c.maxJerk, c.maxVel, c.latAccelMax,
                /*startVel=*/c.startVel);

    //-----------------------------------------
    // PASS 2: Backward pass to meet endVel
    //-----------------------------------------
    backwardPass(samples,
                 c.maxAccel, c.maxDecel, c.maxJerk, c.maxVel, c.latAccelMax,
                 /*endVel=*/c.endVel);

    // Optionally clamp the first velocity again, if needed
    if(samples.front().v < c.startVel) {
        samples.front().v = c.startVel;
        samples.front().a = 0.0;
    }

    //-----------------------------------------
    // PASS 3: Forward pass #2
    //   - Try to accelerate more if the backward pass was too conservative
    //   - We clamp to the velocity found after pass #2
    //-----------------------------------------
    {
      // We'll store the velocities from pass #2 in a temporary array
      std::vector<double> oldV(samples.size());
      for(size_t i=0; i<samples.size(); i++){
          oldV[i] = samples[i].v;
      }
      // Then run forward pass, but clamp velocity to oldV
      forwardPass(samples,
                  c.maxAccel, c.maxJerk, c.maxVel, c.latAccelMax,
                  /*startVel=*/samples[0].v,
                  /*vClampFromPrior=*/1e9 /* will fill below */);

      // Now we must ensure we never exceed the old backward pass velocities:
      for(size_t i=0; i<samples.size(); i++){
          if(samples[i].v > oldV[i]) {
              samples[i].v = oldV[i];
              // recalc acceleration if needed
              // but for simplicity, let's just do a quick fix:
              if(i>0) {
                  double ds_i = samples[i].s - samples[i-1].s;
                  if(ds_i > 1e-9) {
                      double dv = samples[i].v * samples[i].v - samples[i-1].v*samples[i-1].v;
                      samples[i].a = dv / (2.0*ds_i);
                  } else {
                      samples[i].a = samples[i-1].a;
                  }
              }
          }
      }
    }

    //-----------------------------------------
    // PASS 4: Backward pass #2
    //   - Final cleanup to meet endVel exactly
    //   - Also ensure no corner / jerk constraints are violated
    //-----------------------------------------
    {
      // We'll store the velocities from pass #3 in a temp array:
      std::vector<double> oldV(samples.size());
      for(size_t i=0; i<samples.size(); i++){
          oldV[i] = samples[i].v;
      }

      // We'll do a normal backward pass, but clamp to oldV if we are "higher."
      // This ensures we don't accelerate above what pass #3 allowed.
      //    i.e. if pass #3 is lower, keep it
      backwardPass(samples,
                   c.maxAccel, c.maxDecel, c.maxJerk, c.maxVel, c.latAccelMax,
                   c.endVel,
                   /*vClampFromPrior=*/1e9 /* fill below */);

      // clamp to old pass #3 velocities
      for(int i=(int)samples.size()-2; i>=0; i--){
          if(samples[i].v > oldV[i]) {
              samples[i].v = oldV[i];
              // recalc acceleration
              double ds_i = samples[i+1].s - samples[i].s;
              if(ds_i > 1e-9) {
                  double dv = samples[i+1].v*samples[i+1].v - samples[i].v*samples[i].v;
                  samples[i].a = dv / (-2.0*ds_i);
              }
          }
      }
    }

    // If you want to be absolutely sure the first sample is at startVel:
    if(samples.front().v < c.startVel) {
        samples.front().v = c.startVel;
        samples.front().a = 0.0;
        // (Optionally, do another short forward pass, etc.)
    }

    return samples;
}

std::vector<TrajectorySample> convertToTime(const CubicBezier &bz, const std::vector<PathSample> &samples) {
    std::vector<TrajectorySample> traj;
    traj.reserve(samples.size());
    if(samples.empty()) return traj;

    double timeAccum = 0.0;

    for(size_t i = 0; i < samples.size(); i++){
        TrajectorySample ts;
        if(i == 0) {
            ts.t = 0.0;
        } else {
            double ds = samples[i].s - samples[i-1].s;
            double vAvg = 0.5*(samples[i].v + samples[i-1].v);
            double dt = (vAvg < 1e-9) ? 0.0 : (ds / vAvg);
            timeAccum += dt;
            ts.t = timeAccum;
        }

        ts.x = samples[i].x;
        ts.y = samples[i].y;

        double u = samples[i].u;
        Point d1 = bz.getFirstDerivative(u);
        ts.heading = std::atan2(d1.y, d1.x);
        
        ts.v = samples[i].v;
        ts.w = samples[i].kappa * samples[i].v; // approximate

        traj.push_back(ts);
    }

    return traj;
}
