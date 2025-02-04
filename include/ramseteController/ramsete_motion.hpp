// #pragma once
// #include "util/velocity_PID.hpp"          
// #include "trajectory_generator.hpp" // IWYU pragma: keep   
// #include "ramsete_controller.hpp"    



// class RamseteMotion {
// public:
//     /**
//      * The function type for reading the robot's pose (in inches, radians).
//      * If your odometry is in different units, adapt accordingly.
//      */
//     typedef void (*PoseGetter)(double &x, double &y, double &theta);

//     /**
//      * Constructor
//      * @param leftPID       reference to the VelocityPID controlling the left side
//      * @param rightPID      reference to the VelocityPID controlling the right side
//      * @param trackWidth    distance between left and right wheels (in inches)
//      * @param poseFunction  a function pointer that returns the robot's current (x,y,theta)
//      */


//     RamseteMotion(VelocityPID & leftPID,
//                   VelocityPID & rightPID,
//                   double trackWidth,
//                   PoseGetter poseFunction);

//     /**
//      * Follow a cubic Bezier path from param=0..1 using a 
//      * forward-backward velocity profile and a Ramsete loop.
//      * 
//      * @param path      The cubic Bezier
//      * @param reverse   If true, drive the path backward (flips velocity)
//      */
//     void followBezier(const CubicBezier &path, AdvancedConstraints c = {});

// private:
//     VelocityPID & left_;
//     VelocityPID & right_;
//     double trackWidth_;
//     PoseGetter getPose_;

//     RamseteController ramsete_{2.0, 0.7};
// };
