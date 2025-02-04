#include "velocity_PID.hpp"
#include <cmath>
#include <cstdio>

/**
 * Constructor
 */
VelocityPID::VelocityPID(pros::MotorGroup& motorGroup, PIDGains gains,
                         double gearRatio, double wheelDiam)
  : motors_(motorGroup),
    gains_(gains),
    gearRatio_(gearRatio),
    wheelDiameter_(wheelDiam)
{
    // Create a new task. We'll pass "this" as the parameter,
    // and the static function controlLoopTrampoline as the entry point.
    controlTask_ = new pros::Task(controlLoopTrampoline, (void*)this, "VelocityPIDTask");
}

/**
 * Destructor
 */
VelocityPID::~VelocityPID() {
    // If we have a valid task pointer, remove it
    if(controlTask_ != nullptr) {
        controlTask_->remove();
        delete controlTask_;
        controlTask_ = nullptr;
    }
}

/**
 * setTarget
 */
void VelocityPID::setTarget(double velocity) {
    targetVelocity_.store(velocity);
}

/**
 * getCurrentVelocity
 */
double VelocityPID::getCurrentVelocity() const {
    return currentVelocity_;
}

/**
 * The static trampoline to call the member controlLoop()
 */
void VelocityPID::controlLoopTrampoline(void* param) {
    // Cast param back to 'this' pointer
    auto* self = static_cast<VelocityPID*>(param);
    self->controlLoop(); 
}

/**
 * The actual control loop that runs inside the pros::Task
 */
void VelocityPID::controlLoop() {
    // PID state
    double integral = 0.0;
    double lastError = 0.0;
    double lastTarget = 0.0;


    // Starting conditions
    double lastPosition = motors_.get_position(); // in degrees
    uint32_t lastTime = pros::millis();

    // precompute wheel circumference
    double wheelCircumference = M_PI * wheelDiameter_;

    while(true) {
        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        if(dt < 1e-4) {
            // If dt is too small, skip
            pros::delay(10);
            continue;
        }
        lastTime = now;

        // measure current position in degrees
        double positionDeg = motors_.get_position();
        double deltaDeg = positionDeg - lastPosition;
        lastPosition = positionDeg;

        // deg/sec
        double degPerSec = deltaDeg / dt;

        // convert deg/sec to in/s
        double revMotorPerSec  = degPerSec / 360.0;
        double revWheelPerSec  = revMotorPerSec / gearRatio_;
        double inPerSec        = revWheelPerSec * wheelCircumference;
        currentVelocity_       = inPerSec;



        // PID
        double target = targetVelocity_.load(); // in/s
        double acceleration = (target - lastTarget) / dt;
        lastTarget = target;
        
        
        double error  = target - currentVelocity_;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double feedforward = gains_.kV * target + gains_.kA * acceleration;

        double output = feedforward 
                      + gains_.kP * error 
                      + gains_.kI * integral
                      + gains_.kD * derivative;

        // clamp output in [-127..127] for move()
        if(output > 127) output = 127;
        if(output < -127) output = -127;

        // set motor power
        motors_.move((int32_t)std::round(output));

        pros::delay(10); // ~100Hz
    }
}
