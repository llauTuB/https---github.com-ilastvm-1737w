#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "ramseteController/cubic_bezier.hpp"



pros::adi::Encoder vertical_encoder('A', 'B');
pros::adi::Encoder horizontal_encoder('A', 'B');
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    rotation.reset_position();
    optical.set_integration_time(50.0);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
      
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y


            pros::lcd::print(0, "X: %f", vertical_encoder.get_value()); // x
            pros::lcd::print(1, "Y: %f", horizontal_encoder.get_value()); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });    
    
    pros::Task Arm_PID(armPID);
}

void disabled() {}

void competition_initialize() {}

ASSET(curveThree_txt);
void autonomous() {
    CubicBezier path({0,0}, {0,0}, {0,0}, {0,0});
    arm.set_brake_mode(pros::MotorBrake::hold);
    skills();
    // rnc();
    // rpc();
    // bnc();
    // bpc();
    
}


void opcontrol() {
    arm.set_brake_mode(pros::MotorBrake::hold);


    while (true) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.95;
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.95;
        chassis.arcade(leftY, rightX);

        intake.move_voltage((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) - master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) * 12000);
        
        opcontrolLift();

        color_sort_activate();

        mogo_toggle();
        pisun_toggle();
        hang_toggle();
        color_toggle();

        // pros::lcd::print(4, "Lift: %f", arm.get_position() / 100.0);

        pros::delay(25);
    }
}