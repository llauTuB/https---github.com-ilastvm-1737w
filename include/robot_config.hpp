#pragma once
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

extern pros::Controller master;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
 
extern pros::Motor intake;
extern pros::Motor arm;

extern pros::adi::Pneumatics mogo;
extern pros::adi::Pneumatics ziga;
extern pros::adi::Pneumatics intake_lifter;

extern pros::adi::Encoder horizontalEnc;
extern pros::adi::Encoder verticalEnc;

extern pros::Imu imu;