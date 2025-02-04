// #include "ramsete_motion.hpp"
// #include <cmath>
// #include <cstdio>

// // A helper to wrap angles to (-pi, pi)
// static double angleWrap(double ang) {
//     while(ang >  M_PI) ang -= 2*M_PI;
//     while(ang < -M_PI) ang += 2*M_PI;
//     return ang;
// }


// RamseteMotion::RamseteMotion(VelocityPID & leftPID,
//                              VelocityPID & rightPID,
//                              double trackWidth,
//                              PoseGetter poseFunction)
//   : left_(leftPID),
//     right_(rightPID),
//     trackWidth_(trackWidth),
//     getPose_(poseFunction)
// {
// }

// void RamseteMotion::followBezier(const CubicBezier &path, AdvancedConstraints c) {

//     int steps = 500;

//     // build path samples
//     auto pathSamples = generateVelocityProfile(
//         path, 
//         c,
//         steps
//     );

//     // 2) convert to time-based
//     auto trajectory = convertToTime(path, pathSamples);

//     // 3) We'll run a loop from t=0..end
//     int startTime = pros::millis();
//     size_t idx = 0;

//     while(idx < trajectory.size()) {
//         // current time
//         int now = pros::millis();
//         double tElapsed = (now - startTime)/1000.0;

//         // If we've surpassed the current sample's time, move to next
//         if(idx < trajectory.size()-1 && tElapsed > trajectory[idx].t) {
//             idx++;
//         }

//         // get reference
//         double xRef     = trajectory[idx].x;
//         double yRef     = trajectory[idx].y;
//         double thetaRef = trajectory[idx].heading;
//         double vRef     = trajectory[idx].v;
//         double wRef     = trajectory[idx].w;

//         if(c.reverse) {
//             // invert velocities & shift heading by pi
//             vRef     = -vRef;
//             wRef     = -wRef;
//             thetaRef += M_PI;
//             thetaRef  = angleWrap(thetaRef);
//         }


//         double xRobot, yRobot, thetaRobot;
//         getPose_(xRobot, yRobot, thetaRobot);

//         // run ramsete
//         auto [vCmd, wCmd] = ramsete_.calculate(
//             xRobot, yRobot, thetaRobot,
//             xRef,   yRef,   thetaRef,
//             vRef,   wRef
//         );

//         // convert (vCmd, wCmd) to left/right in/s
//         double vLeft  = vCmd - 0.5*wCmd*trackWidth_;
//         double vRight = vCmd + 0.5*wCmd*trackWidth_;

//         // set velocity on each side
//         left_.setTarget(vLeft);
//         right_.setTarget(vRight);

//         // check if done
//         if (idx >= trajectory.size() - 1) {
//             double dx = xRef - xRobot;
//             double dy = yRef - yRobot;
//             double dTheta = angleWrap(thetaRef - thetaRobot);
//             if (std::hypot(dx, dy) < 2.0 && std::abs(dTheta) < 0.1) { // Within 2in, 0.1rad
//                 break;
//             }
//         }

//         pros::delay(10); // ~ 50 Hz loop
//     }

//     // Done: set velocity to 0
//     left_.setTarget(0.0);
//     right_.setTarget(0.0);
// }
