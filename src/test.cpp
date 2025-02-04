#include <cmath>
#include <vector>
#include "lemlib/chassis/chassis.hpp"
#include "config/lemlib_config.hpp"
#include "ramseteController/ramsete_controller.hpp"
#include "util/velocity_PID.hpp"


static double angleWrap(double ang) {
    while(ang >  M_PI) ang -= 2*M_PI;
    while(ang < -M_PI) ang += 2*M_PI;
    return ang;
}


void lemlib::Chassis::followBezier(const CubicBezier &path, AdvancedConstraints ramseteParams, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { followBezier(path, ramseteParams, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    int steps = 500;

    // build path samples
    auto pathSamples = generateVelocityProfile(
        path, 
        ramseteParams,
        steps
    );

    // 2) convert to time-based
    auto trajectory = convertToTime(path, pathSamples);

    // 3) We'll run a loop from t=0..end
    int startTime = pros::millis();
    size_t idx = 0;
    distTraveled = 0;
    Pose currPose = this->getPose(true);
    Pose lastPose = currPose;

    while(idx < trajectory.size()) {
        // current time
        int now = pros::millis();
        double tElapsed = (now - startTime)/1000.0;

        // If we've surpassed the current sample's time, move to next
        if(idx < trajectory.size()-1 && tElapsed > trajectory[idx].t) {
            idx++;
        }

        // get reference
        double xRef     = trajectory[idx].x;
        double yRef     = trajectory[idx].y;
        double thetaRef = trajectory[idx].heading;
        double vRef     = trajectory[idx].v;
        double wRef     = trajectory[idx].w;

        if(ramseteParams.reverse) {
            // invert velocities & shift heading by pi
            vRef     = -vRef;
            wRef     = -wRef;
            thetaRef += M_PI;
            thetaRef  = angleWrap(thetaRef);
        }


        double xRobot, yRobot, thetaRobot;
        currPose = this->getPose(true);
        distTraveled += currPose.distance(lastPose);
        lastPose = currPose;


        xRobot = currPose.x;
        yRobot = currPose.y;
        thetaRobot = currPose.theta;

        // run ramsete
        auto [vCmd, wCmd] = ramsete.calculate(
            xRobot, yRobot, thetaRobot,
            xRef,   yRef,   thetaRef,
            vRef,   wRef
        );

        // convert (vCmd, wCmd) to left/right in/s
        double vLeft  = vCmd - 0.5*wCmd*drivetrain.trackWidth;
        double vRight = vCmd + 0.5*wCmd*drivetrain.trackWidth;

        // set velocity on each side
        leftP.setTarget(vLeft);
        rightP.setTarget(vRight);

        // check if done
        if (idx >= trajectory.size() - 1) {
            double dx = xRef - xRobot;
            double dy = yRef - yRobot;
            double dTheta = angleWrap(thetaRef - thetaRobot);
            if (std::hypot(dx, dy) < 2.0 && std::abs(dTheta) < 0.1) { // Within 2in, 0.1rad
                break;
            }
        }

        pros::delay(10); // ~ 50 Hz loop
    }

    // Done: set velocity to 0
    leftP.setTarget(0.0);
    rightP.setTarget(0.0);

    distTraveled = -1;
    // give the mutex back
    this->endMotion();
}