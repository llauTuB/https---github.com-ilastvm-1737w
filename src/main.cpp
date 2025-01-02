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



bool isColorActive = false;

const double HUE_BLUE_MIN = 200; 
const double HUE_BLUE_MAX = 260; 

void color_sorter_task() {
    while (true) {
        
        double hue = optical.get_hue();

        if (hue >= HUE_BLUE_MIN && hue <= HUE_BLUE_MAX) {
            intake_lifter.set_value(127); 
        } else {
            intake_lifter.set_value(-127); 
        }

        pros::delay(50); 
    }
}

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
                
                float power = Arm_lift.update(liftTarget - arm.get_position());
                power = fmin(fmax(power, -127), 127);
                arm.move(power);

                
                if (std::abs(liftTarget - arm.get_position()) < 1.2) {
                    arm.move_voltage(0);
                    liftPidActive = false;
                }
            } else {
                pros::delay(20); 
            }
        }
    });

    optical.set_led_pwm(100);
    pros::Task colorSorter(color_sorter_task);

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
ASSET(curveThree_txt);
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    arm.set_brake_mode(pros::MotorBrake::hold);
    chassis.setPose(-62, 0, 90);

   



//1ST PART!!!
    intake.move_voltage( 12000);
    pros::delay(500);
    chassis.moveToPoint(-48, 0, 950, {true, 120, 10, 1});
    chassis.turnToPoint(-48, -24, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1});
    chassis.moveToPoint(-48, 24, 950,{false,80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    
    //1st ring
    chassis.turnToPoint(-24, 22, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-24, 22, 1000, {true, 80, 10, 1});

    //2nd ring
    chassis.turnToPoint(24, 47, 310, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 120, 10, 1});
    // chassis.turnToHeading(90, 600, {AngularDirection::AUTO, 127, 10, 1});

    chassis.moveToPoint(24, 47, 1600, {true, 120, 10, 1});
    pros::delay(1500);

    //wallstake
    chassis.turnToPoint(-22, 68, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(3, 53, 950, {true, 120, 10, 1});
    chassis.waitUntil(20);
    moveArmToPosition(49);
    // chassis.turnToPoint(0, 96, 700, {true, lemlib::AngularDirection::CW_CLOCKWISE, 120, 10, 1} );
    chassis.turnToHeading(1, 800, {AngularDirection::AUTO, 110, 10, 1});
    chassis.moveToPoint(3, 56, 600, {true, 30, 10, 1});
    chassis.waitUntilDone();
    pros::delay(800);
    intake.brake();
    moveArmToPosition(340);
    pros::delay(500);
    
    //back after wallstake
    chassis.moveToPoint(3, 38, 950, {false, 110, 10, 1});
    chassis.waitUntilDone();
    
    intake.move_voltage( -12000);
    moveArmToPosition(0);
    pros::delay(800);
    intake.move_voltage( 12000);
    chassis.moveToPoint(3, 45, 550, {true, 120, 10, 1});

    //3th ring
    chassis.turnToPoint(-48, 46, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-24, 48, 950, {true, 80, 10, 1});
    pros::delay(500);

    //4th ring
    chassis.moveToPoint(-48, 46, 950, {true, 50, 10, 1});
    pros::delay(450);

    //5th ring
    chassis.moveToPoint(-56, 46, 950, {true, 50, 10, 1});
    pros::delay(700);
    
    //6th ring
    chassis.turnToPoint(-36, 70, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-48, 56, 950, {true, 80, 10, 1});
    pros::delay(950);

    //mogo in the corner
    chassis.turnToPoint(0, 60,450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-62, 60, 700, {false, 50, 10, 1});
    chassis.turnToPoint(0, 0, 500, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    //chassis.moveToPoint(-62, 62, 700, {false, 40, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(0);
    intake.brake();
    
    



//2ND PART!!!
    //2nd mogo
    chassis.moveToPoint(-48, 48, 950, {true, 105, 10, 1});
    chassis.turnToPoint(-48, 72, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-50, -22, 5000, {false, 80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    intake.move_voltage( 12000);

    //1st ring
    chassis.turnToPoint(0, -21, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-26, -21, 1000, {true, 110, 10, 1});

    // //2nd ring
    // chassis.turnToPoint(0, 24, 700, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    // chassis.moveToPoint(0, 0, 1500, {true, 100, 10, 1});
    // pros::delay(200);
    // chassis.moveToPoint(-24, -22, 1500, {false, 100, 10, 1});

    //2nd ring 
    chassis.turnToPoint(-26, -72, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-26, -43, 950, {true, 105, 10, 1});

    //3rd ring
    chassis.turnToPoint(-72, -43, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-48, -43, 950, {true, 105, 10, 1});
    pros::delay(700);

    //4th ring
    chassis.moveToPoint(-59, -43, 950, {true, 50, 10, 1});
    pros::delay(700);

    //5th ring
    chassis.turnToPoint(-48, -70, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-48, -56, 950, {true, 50, 10, 1});
    pros::delay(1000);

    //mogo in the corner
    chassis.turnToPoint(0, -24, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-64, -64, 1200, {false, 70, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(0);
    

    //wallstake
    //chassis.turnToPoint(0, -56, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.turnToHeading(83, 500, {AngularDirection::AUTO, 120, 10, 1});

    chassis.moveToPoint(-5, -56, 2200, {true, 90, 10, 1});
    chassis.waitUntil(20);
    moveArmToPosition(49);
    // chassis.turnToPoint(0, -72, 700, {true, lemlib::AngularDirection::CW_CLOCKWISE, 120, 10, 1} );
    chassis.turnToHeading(181.5, 600, {AngularDirection::AUTO, 120, 10, 1});
    chassis.moveToPoint(-5, -63, 1000, {true, 80, 10, 1});
    chassis.waitUntilDone();
    pros::delay(1000);
    intake.brake();
    moveArmToPosition(340);
    pros::delay(500);
    
    //back after wallstake
    chassis.moveToPoint(-5, -42, 1000, {false, 80, 10, 1});
    chassis.waitUntilDone();
    

//3RD PART!!!
    //1st ring
    intake.move_voltage( -12000);
    moveArmToPosition(0);
    pros::delay(800);
    intake.move_voltage( 12000);


    chassis.turnToPoint(48, 0, 500, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(24, -24, 950, {true, 105, 10, 1});
    chassis.waitUntilDone();
    intake.brake();


    //mogo
    chassis.turnToPoint(48, 0, 750, {false, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );

    chassis.follow(curveThree_txt, 10, 2000, false);
    chassis.waitUntilDone();
    mogo.set_value(true);
    intake.move_voltage( 12000);
    


    //2nd 

    chassis.turnToPoint(47, -68, 500, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(47, -50, 1500, {true, 100, 10, 1});

 

    //3rd ring
    chassis.moveToPoint(47, 0, 2000, {false, 120, 10, 1});
    chassis.turnToPoint(-2, 48, 600, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.turnToHeading(315, 650, {AngularDirection::AUTO, 120, 10, 1});

    chassis.moveToPoint(21, 25, 1000, {true, 110, 10, 1});

    //4th ring
    chassis.turnToPoint(72, 72, 500, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(45, 45, 950, {true, 80, 10, 1});

    //5th ring
    chassis.moveToPoint(38, 38, 950, {false, 110, 10, 1});
    chassis.turnToPoint(48, 60, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(43, 50, 700, {true, 100, 10, 1});
    pros::delay(600);




    //in the corner
    // chassis.moveToPoint(39, 50, 950, {false, 100, 10, 1});
    chassis.turnToHeading(50, 500, {AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(28, 52, 1000, {false, 70, 10, 1});
    chassis.waitUntilDone();
    ziga.set_value(1);
    chassis.moveToPoint(64, 65, 3000, {true, 70, 10, 1});
    chassis.turnToHeading(225, 1200, {AngularDirection::AUTO, 100, 10, 1});

    chassis.moveToPoint(67, 66, 1000, {false, 90, 10, 1});
    mogo.set_value(0);
    chassis.waitUntilDone();
    ziga.set_value(0);
    
    



    //2nd mogo in the corner
    
    chassis.moveToPoint(40, 40, 1000, {true, 80, 10, 1});
    chassis.turnToPoint(66, -24, 500, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(66, -15, 3200, {true, 110, 10, 1});
    chassis.moveToPoint(79, -61, 3000, {true, 120, 10, 1});
    chassis.moveToPoint(-60, -61, 3000, {false, 70, 10, 1});
    
    intake.brake();











    // chassis.turnToPoint(85, -70, 1000, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    // chassis.turnToHeading(140, 1500, {AngularDirection::CCW_COUNTERCLOCKWISE, 100, 10, 1});

    // chassis.waitUntilDone();
    // ziga.set_value(1);
    // chassis.moveToPoint(64, -54, 1500, {true, 40, 10, 1});
    // chassis.turnToHeading(315, 950, {AngularDirection::AUTO, 100, 10, 1});
    // chassis.waitUntilDone();
    // mogo.set_value(0);



































}





/**
 * Runs in driver _control
 */

 bool mogoState = false; 
 bool pisunState = false;
 bool intake_l_State = false;

void toggleIntakeLifter() {
    intake_l_State = !intake_l_State;  
    if (intake_l_State) {
        intake_lifter.set_value(127);  
    } else {
        intake_lifter.set_value(-127); 
    }
}
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

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pisunState = !pisunState;
            ziga.set_value(pisunState);
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            toggleIntakeLifter(); 
            pros::delay(100);  
        }

        // pros::lcd::print(4, "Lift: %f", arm.get_position() / ratio);

        // delay to save resources
        pros::delay(25);
    }
}
