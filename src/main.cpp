#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot_config.hpp"




lemlib::PID Arm_lift(
   0.87,
   0,
   0.2,
   0,
   false 
);



bool liftPidActive = false;
double liftTarget;
double const ratio = 5.0;

void moveArmToPosition(double target) {
    liftPidActive = true;
    target *= ratio;
    liftTarget = target;
}

double initPosArm = 0.0;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });    
    
    pros::Task armPID([]() {
        while (true) {
            if (liftPidActive) {
                // Рассчитываем мощность на основе ошибки
                float power = Arm_lift.update(liftTarget - arm.get_position());
                power = fmin(fmax(power, -127), 127);
                arm.move(power);

                // Условие завершения PID
                if (std::abs(liftTarget - arm.get_position()) < 1.2) {
                    arm.move_voltage(0);
                    liftPidActive = false;
                }
            } else {
                pros::delay(20); // Пауза, чтобы не перегружать процессор
            }
        }
    });
}



/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(curveOne_txt); // '.' replaced with "_" to make c++ happy
ASSET(curveTwo_txt); 
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    arm.set_brake_mode(pros::MotorBrake::hold);

    chassis.setPose(-62, 0, 90);
    intake.move_voltage( 12000);
    pros::delay(500);
    chassis.moveToPoint(-48, 0, 1000, {true, 100, 10, 1});
    chassis.turnToPoint(-48, -24, 750, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1});
    chassis.moveToPoint(-48, 24, 1000,{false,80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);

    chassis.turnToPoint(-24, 24, 750, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-24, 24, 1000, {true, 80, 10, 1});  
    chassis.turnToPoint(24, 48, 750, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1});

    chassis.moveToPoint(24, 48, 2000, {true, 100, 10, 1});
    pros::delay(500);
    chassis.turnToPoint(0, 41, 750, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(0, 41, 1000, {true, 100, 10, 1});
    chassis.turnToPoint(-1, 72, 800, {true, lemlib::AngularDirection::CW_CLOCKWISE, 80, 10, 1} );
    chassis.waitUntilDone();
    moveArmToPosition(55);
    chassis.moveToPoint(-1, 60, 1000, {true, 70, 10, 1});
    chassis.waitUntilDone();
    pros::delay(1500);
    intake.brake();
    moveArmToPosition(400);
    pros::delay(800);
    
    
    chassis.moveToPoint(-1, 48, 1000, {true, 100, 10, 1});
    chassis.waitUntilDone();
    moveArmToPosition(0);
    intake.move_voltage( 12000);
    chassis.turnToPoint(-24, 48, 750, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-24, 48, 1000, {true, 100, 10, 1});
    pros::delay(400);
    chassis.moveToPoint(-48, 48, 1000, {true, 80, 10, 1});
    pros::delay(400);
    chassis.moveToPoint(-48, 48, 1000, {true, 80, 10, 1});
    pros::delay(400);
    chassis.moveToPoint(-63, 48, 1000, {true, 80, 10, 1});
    pros::delay(200);

 
    chassis.follow(curveOne_txt, 10, 5000, false);

    chassis.moveToPoint(-48, 64, 700, {true, 80, 10, 1});
    pros::delay(600);


    chassis.moveToPoint(-48, 48, 700, {false, 80, 10, 1});
    chassis.turnToPoint(-24, 24, 750, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-65, 65, 700, {false, 80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(0);
    intake.brake();
    chassis.follow(curveTwo_txt, 10, 5000, true);




    
    // chassis.waitUntil(10);
    // intake.brake();
    // move 48" forwards







    // // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPose(20, 15, 90, 4000);

    // chassis.setPose(-68.763, -23.197, 68.205);
    // chassis.follow(curve_txt, 5, 4000, true);
    // chassis.moveToPoint(0, 10, 4000);
    // // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // // cancel the movement after it has traveled 10 inches
    // chassis.waitUntil(10);
    // chassis.cancelMotion();
    // // Turn to face the point x:45, y:-45. Timeout set to 1000 
    // // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // // Turn to face a direction of 90º. Timeout set to 1000
    // // will always be faster than 100 (out of a maximum of 127)
    // // also force it to turn clockwise, the long way around
    // chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // // following the path with the back of the robot (forwards = false)
    // // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // // the movement will run immediately
    // // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    // pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // // wait until the movement is done
    // chassis.waitUntilDone();
    // pros::lcd::print(4, "pure pursuit finished!");
}





/**
 * Runs in driver _control
 */

 bool mogoState = false; 
 bool pisunState = false;
 bool intake_l_State = false;

void opcontrol() {

    arm.set_brake_mode(pros::MotorBrake::hold); 


    // master
    // loop to continuously update motors
    while (true) {


        // get joystick positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.95;
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.95;


        chassis.arcade(leftY, rightX);

        intake.move_voltage((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) - master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) * 12000);
       

        // if (!liftPidActive) {
        //     arm.move_voltage((master.get_digital(pros::E_CONTROLLER_DIGITAL_B) - master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) * 12000);
        // } else {
        //     arm.move_voltage(0);  
        // }


                if (!liftPidActive) {
            arm.move_voltage((master.get_digital(pros::E_CONTROLLER_DIGITAL_B) - 
                              master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) * 12000);
        }


        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            moveArmToPosition(50);
            
        }
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            moveArmToPosition(340);  
            
        } 

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            moveArmToPosition(initPosArm);    
        } 



    //  if(liftPidActive){
    //   // set target
    //   float power = Arm_lift.update(liftTarget - arm.get_position());
    //   power = fmin(fmax(power, -127), 127);
    //   // move lift based on output
    //   arm.move(power);    
      
    //    if (std::abs(liftTarget - arm.get_position()) < 1.2) {
    //             arm.move_voltage(0);
    //             liftPidActive = false;
    //     }
    // }


        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            mogoState = !mogoState;  
            mogo.set_value(mogoState);  
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            pisunState = !pisunState;
            ziga.set_value(pisunState);
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intake_l_State = !intake_l_State;  
            intake_lifter.set_value(intake_l_State);  
        }

        // pros::lcd::print(4, "Lift: %f", arm.get_position() / ratio);

        // delay to save resources
        pros::delay(25);
    }
}
