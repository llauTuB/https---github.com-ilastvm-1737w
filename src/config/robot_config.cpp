#include "main.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// master
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-13, -14, -15},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({16, 17, 18}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//intake
pros::Motor intake(12, pros::MotorGearset::blue);

//arm
pros::Motor arm(-21, pros::MotorGearset::green);

// Inertial Sensor on port 1
pros::Imu imu(1);

// Optical sensor
pros::Optical optical(4);

//Rotation sensor
pros::Rotation rotaion(3);

//Mogomech
pros::adi::Pneumatics mogo('a',false);

//Ziga
pros::adi::Pneumatics ziga('d', false);

//intake
pros::adi::Pneumatics color_sort('c', false);

//Hang
pros::adi::Pneumatics hang('b', false);

// // tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::adi::Encoder horizontalEnc('E', 'F', true);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::adi::Encoder verticalEnc('G', 'H', false);