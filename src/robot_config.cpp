#include "main.h" // IWYU pragma: keep

// master
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, -12, -13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//intake
pros::Motor intake(17, pros::MotorGearset::blue);

//arm
pros::Motor arm(-21, pros::MotorGearset::green);

// Inertial Sensor on port 10
pros::Imu imu(1);

//Mogomech
pros::adi::Pneumatics mogo('a',false);

//Ziga
pros::adi::Pneumatics ziga('b', false);

//intake
pros::adi::Pneumatics intake_lifter('c', true);


// // tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::adi::Encoder horizontalEnc('G', 'H', true);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::adi::Encoder verticalEnc('E', 'F', false);