#pragma once
#include "pros/motor_group.hpp"
#include <atomic>


struct PIDGains {
    double kP = 0.0; 
    double kI = 0.0; 
    double kD = 0.0;
    double kV = 0.0; 
    double kA = 0.0;
};

/**
 * A simple velocity PID class that runs in a background thread.
 */
class VelocityPID {
public:
    /**
     * Constructor
     * @param motorGroup   The motors to be controlled as one side
     * @param gains        PID gains
     * @param gearRatio    motorRev:wheelRev ratio
     * @param wheelDiam    wheel diameter in inches (to compute circumference)
     */
    VelocityPID(pros::MotorGroup& motorGroup, const PIDGains gains = {}, 
                double gearRatio = 48.0/36.0, double wheelDiam = 3.25);

    /**
     * Destructor
     */
    ~VelocityPID();

    /**
     * Set the target velocity in in/s
     */
    void setTarget(double velocity);

    /**
     * Get the current measured velocity in in/s
     */
    double getCurrentVelocity() const;

private:
    // private data
    pros::MotorGroup& motors_;
    PIDGains gains_;
    double gearRatio_;
    double wheelDiameter_;

    std::atomic<double> targetVelocity_{0.0}; // in/s
    double currentVelocity_ = 0.0;


    pros::Task* controlTask_ = nullptr;

    // A static helper to pass into pros::Task
    static void controlLoopTrampoline(void* param);

    /**
     * The control loop that runs in a background thread
     */
    void controlLoop();
};
