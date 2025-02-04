#include "main.h" // IWYU pragma: keep



void get_pose(double &x, double &y, double &theta) {
   x=chassis.getPose().x; 
   y=chassis.getPose().y; 
   theta=chassis.getPose(true).theta;
}

 void wait_red(){
    uint32_t start = pros::millis();
    // intake.move_voltage(12000);
    while(pros::millis() - start < 2000){
        if(optical.get_hue() >= 0 && optical.get_hue() <= 20){
            pros::delay(400);
            intake.brake();
            return;
        }
        pros::delay(10);
    }
    intake.brake();
 }

void skiils() {

chassis.setPose(-62, 0, 90);
optical.set_led_pwm(100);
// pros::Task colorSorter(color_sorter_red);
//1ST PART!!!
    intake.move_voltage( 12000);
    pros::delay(600);
    chassis.moveToPoint(-48, 0, 950, {true, 120, 10, 1});
    chassis.turnToPoint(-48, -24, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1});
    chassis.moveToPoint(-48, 24, 950,{false,80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    intake.move_voltage( -12000);
    pros::delay(300);
    intake.move_voltage( 12000);

    
    //1st ring
    chassis.turnToPoint(-24, 21, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-26, 21, 1000, {true, 80, 10, 1});

    //2nd ring
    chassis.turnToPoint(24, 47, 400, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 120, 10, 1});
    // chassis.turnToHeading(90, 600, {AngularDirection::AUTO, 127, 10, 1});

    chassis.moveToPoint(24, 47, 1600, {true, 120, 10, 1});
    pros::delay(1700);

    //wallstake
    chassis.turnToPoint(-22, 68, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(1.5, 40, 950, {true, 120, 10, 1});
    chassis.waitUntil(20);
    moveArmToPosition(48);
    // chassis.turnToPoint(0, 96, 700, {true, lemlib::AngularDirection::CW_CLOCKWISE, 120, 10, 1} );
    chassis.turnToHeading(1.7, 800, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(0.5, 58.5, 1300, {true, 40 , 10, 1});
    chassis.waitUntilDone();
    wait_red();
    moveArmToPosition(350);
    pros::delay(550);
    
    //back after wallstake
    chassis.moveToPoint(0.5, 38, 950, {false, 110, 10, 1});
    chassis.waitUntilDone();
    
    intake.move_voltage( -12000);
    moveArmToPosition(0);
    pros::delay(400);
    intake.move_voltage( 12000);
    chassis.moveToPoint(2, 45, 550, {true, 120, 10, 1});

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
    chassis.moveToPoint(-60, 60, 720, {false, 50, 10, 1}); 
    chassis.waitUntil(5);
    mogo.set_value(0);
    chassis.turnToPoint(0, 0, 500, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    //chassis.moveToPoint(-62, 62, 700, {false, 40, 10, 1});

   intake.brake(); 
   
    
    
//2ND PART!!!
    //2nd mogo
    chassis.moveToPoint(-48, 48, 1000, {true, 105, 10, 1});
    chassis.turnToPoint(-48, 72, 460, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-48, -23, 5000, {false, 80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    intake.move_voltage( -12000);
    pros::delay(300);
    intake.move_voltage( 12000);

    //1st ring
    chassis.turnToPoint(0, -18, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-26, -18, 1000, {true, 110, 10, 1});

    // //2nd ring
    // chassis.turnToPoint(0, 24, 700, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    // chassis.moveToPoint(0, 0, 1500, {true, 100, 10, 1});
    // pros::delay(200);
    // chassis.moveToPoint(-24, -22, 1500, {false, 100, 10, 1});

    //2nd ring 
    chassis.turnToPoint(-26, -72, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-26, -43, 1050, {true, 100, 10, 1});

    //3rd ring
    chassis.turnToPoint(-72, -43, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-48, -43, 1000, {true, 100, 10, 1});
    pros::delay(700);

    //4th ring
    chassis.moveToPoint(-55, -42, 950, {true, 50, 10, 1});
    pros::delay(700);

    //5th ring
    chassis.turnToPoint(-48, -70, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-48, -55, 950, {true, 50, 10, 1});
    pros::delay(1000);

    //mogo in the corner
    chassis.turnToPoint(0, -24, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(-64, -64, 1000, {false, 80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(0);
    intake.move_voltage( -12000);
    pros::delay(150);
    intake.move_voltage( 12000);
    

    //wallstake
    chassis.turnToHeading(80, 490, {lemlib::AngularDirection::AUTO, 127, 10, 1});

    chassis.moveToPoint(-4, -48, 2000, {true, 100, 10, 1});
    chassis.waitUntil(20);
    moveArmToPosition(48);
    // chassis.turnToPoint(0, -72, 700, {true, lemlib::AngularDirection::CW_CLOCKWISE, 120, 10, 1} );
    chassis.turnToHeading(182, 600, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(-3.5, -68, 1100, {true, 50, 10, 1});
    chassis.waitUntilDone();
    wait_red();
    moveArmToPosition(380);
    pros::delay(550);
    
    //back after wallstake
    chassis.moveToPoint(-4, -50, 800, {false, 127, 10, 1});
    chassis.waitUntilDone();
    

//3RD PART!!!
    //1st ring
    intake.move_voltage( -12000);
    moveArmToPosition(0);
    pros::delay(700);
    intake.move_voltage( 12000);


    chassis.turnToPoint(48, 0, 500, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(25, -26, 950, {true, 127, 10, 1});
    chassis.waitUntilDone();
    intake.brake();


    //mogo
    // chassis.turnToPoint(48, 0, 820, {false, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.turnToHeading(225, 900, {lemlib::AngularDirection::AUTO, 127, 10, 1});

    // chassis.follow(curve4_txt, 7, 2500, false);
    chassis.moveToPoint(48, 0, 3000, {false, 70, 8, 1});
    chassis.waitUntilDone();
    mogo.set_value(true);
    intake.move_voltage(-12000);
    pros::delay(100);
    intake.move_voltage( 12000);
    
    //2nd 
    //chassis.turnToPoint(47, -68, 500, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(43, -8, 400, {true, 127, 10, 1});
    chassis.turnToHeading(180, 650, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(40, -48, 1500, {true, 100, 10, 1});

    //3rd ring
    chassis.moveToPoint(42, 0, 2000, {false, 120, 10, 1});
    // chassis.turnToPoint(-2, 48, 600, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.turnToHeading(300, 600, {lemlib::AngularDirection::AUTO, 127, 10, 1});

    chassis.moveToPoint(17, 20, 1100, {true, 110, 10, 1});

    //4th ring
    chassis.turnToPoint(72, 72, 500, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(41, 40, 860, {true, 100, 10, 1});

    //5th ring
    chassis.moveToPoint(35, 35, 950, {false, 110, 10, 1});
    chassis.turnToPoint(48, 65, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(43, 51  , 700, {true, 127, 50, 1});
    pros::delay(200);


    //in the corner
    // chassis.moveToPoint(39, 50, 950, {false, 100, 10, 1});
    chassis.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(20, 52, 1000, {false, 120, 10, 1});
    chassis.waitUntilDone();
    ziga.set_value(1);
    chassis.moveToPoint(67, 67, 1500, {true, 120, 20, 1});
    chassis.turnToHeading(225, 1200, {lemlib::AngularDirection::AUTO, 120, 10, 1});

    chassis.moveToPoint(68, 66, 1000, {false, 90, 10, 1});
    mogo.set_value(0);
    chassis.waitUntilDone();
    ziga.set_value(0);
    
    //2nd mogo in the corner
    chassis.moveToPoint(40, 40, 1000, {true, 100, 10, 1});
    chassis.turnToPoint(60, -24, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    chassis.moveToPoint(60, -15, 1500, {true, 127, 50, 1});
    chassis.moveToPoint(72, -61, 1400, {true, 127, 127, 1});
    intake.brake();
    chassis.turnToPoint(-24, 30, 500, {false, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1} );
    hang.set_value(1); 
    chassis.moveToPoint(0, 17, 1600, {false, 92, 92, 1});
    chassis.moveToPoint(72, -55, 400, {true, 30, 10, 1});
}

void rnc() {
    // rnc
    optical.set_led_pwm(100);
    pros::Task colorSorter(color_sorter_red);
    chassis.setPose(-56, 13, 220);
    // moveArmToPosition(50);
    // intake.move_voltage(12000);
    // pros::delay(800);
    // intake.brake();
    // moveArmToPosition(500);
    // pros::delay(800);SSSS
    // moveArmToPosition(0);
    // pros::delay(800);
    // intake.move_voltage(12000);
    chassis.moveToPoint(-21, 24, 1500,{false,80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    pros::delay(500);
    intake.move_voltage(12000); //удалить потом
    chassis.turnToPoint(-14, 39, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1});
    chassis.moveToPoint(-14, 39, 950, {true, 70, 10, 1});
    chassis.moveToPoint(-14.5, 58, 950, {true, 100, 10, 1});
    pros::delay(500);
    chassis.turnToPoint(-25, 51, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-25, 51, 950, {true, 90, 10, 1});
    pros::delay(1000);
    // chassis.turnToPoint(-48, 18, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-48, 18, 1500,{true,100, 10, 1});
    intake.move_voltage(12000);
    chassis.turnToPoint(-48, -9, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.waitUntilDone();
    chassis.moveToPoint(-48, 9, 1500,{true,50, 10, 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(-48, -12, 1500,{true,120, 10, 1});
}

void rpc() {
    optical.set_led_pwm(100);
    pros::Task colorSorter(color_sorter_red);
    chassis.setPose(-56, -13, 310);
    // moveArmToPosition(50);
    // intake.move_voltage(12000);
    // pros::delay(800);
    // intake.brake();
    // moveArmToPosition(500);
    // pros::delay(800);
    // moveArmToPosition(0);
    // pros::delay(800);
    intake.move_voltage(12000);
    chassis.moveToPoint(-48, -20, 1500, {false, 120, 10, 1});
    chassis.turnToPoint(-48, 20, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1}); 
    moveArmToPosition(0);
    chassis.moveToPoint(-48, 10, 950, {true, 70, 10, 1});
    chassis.turnToPoint(-20, -30, 450, {false, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    pros::delay(1000);
    intake.move_voltage(0);
    chassis.moveToPoint(-18, -30, 950, {false, 90, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    pros::delay(500);
    intake.move_voltage(12000);
    chassis.turnToPoint(-26, -50, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-26, -42, 950, {true, 80, 10, 1});
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.turnToPoint(-17, -39, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    mogo.set_value(0);
    chassis.moveToPoint(-15, -42, 950, {true, 80, 10, 1});
    chassis.waitUntilDone();
    ziga.set_value(1);
    pros::delay(500);
    chassis.moveToPoint(-48, -38, 950, {false, 120, 10, 1});
    chassis.waitUntilDone();
    ziga.set_value(0);
    chassis.turnToPoint(-24, 2, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-25, 2, 1500, {true, 60, 10, 1});
}

void bnc() {
    optical.set_led_pwm(100);
    pros::Task colorSorter(color_sorter_blue);
    chassis.setPose(56, 13, 140);
    // moveArmToPosition(47);
    // intake.move_voltage(12000);
    // pros::delay(800);
    // intake.brake();
    // moveArmToPosition(500);
    // pros::delay(800);
    // moveArmToPosition(0);
    // pros::delay(800);
    chassis.moveToPoint(21, 24, 2000, {false, 80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    pros::delay(500);
    intake.move_voltage(12000);
    chassis.turnToPoint(15, 34, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1});
    chassis.moveToPoint(15, 34, 950, {true, 70, 10, 1});
    chassis.moveToPoint(18, 68, 950, {true, 70, 10, 1});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToPoint(24, 46, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1});
    chassis.moveToPoint(24, 46, 950, {true, 90, 10, 1});
    pros::delay(1000);
    chassis.moveToPoint(46, 20, 1500, {true, 100, 10, 1});
    intake.move_voltage(10000);
    chassis.turnToPoint(46, -9, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(46, 10, 1500, {true, 40, 10, 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(46, -17, 1500, {true, 100, 10, 1});
    pros::delay(1000);
    chassis.turnToPoint(-60, -59, 450, {true, lemlib::AngularDirection::CW_CLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(-60, -59, 1500,{true,50, 10, 1});
}

void bpc() {
    optical.set_led_pwm(100);
    pros::Task colorSorter(color_sorter_blue);
    chassis.setPose(56, -13, 50);
    moveArmToPosition(49);
    intake.move_voltage(12000);
    pros::delay(800);
    intake.brake();
    moveArmToPosition(500);
    pros::delay(800);
    moveArmToPosition(0);
    pros::delay(900);
    intake.move_voltage(12000);
    chassis.moveToPoint(46, -21, 1500, {false, 120, 10, 1});
    chassis.turnToPoint(48, 25, 450, {true, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 
    chassis.moveToPoint(52, 18, 950, {true, 80, 10, 1});
    pros::delay(800);
    chassis.moveToPoint(52, 5, 950, {false, 100, 10, 1});
    chassis.turnToPoint(24, -24, 600, {false, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1}); 

    intake.move_voltage(0);
    chassis.moveToPoint(20, -23, 1400, {false, 80, 10, 1});
    chassis.waitUntilDone();
    mogo.set_value(1);
    pros::delay(100);
    intake.move_voltage(12000);
    chassis.moveToPoint(26, -20, 1000, {true, 80, 10, 1});
    chassis.turnToHeading(180, 1000, {lemlib::AngularDirection::AUTO, 127, 10, 1});

    chassis.moveToPoint(23, -39, 950, {true, 120, 10, 1});
    chassis.moveToPoint(23, -32, 950, {false, 100, 10, 1});
    chassis.waitUntilDone();
    pros::delay(300);
    mogo.set_value(0);
    chassis.turnToHeading(275, 800, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(17, -33, 950, {true, 100, 10, 1});
    chassis.waitUntilDone();
    ziga.set_value(1);
    pros::delay(100);
    chassis.moveToPoint(48, -36, 1200, {false, 40, 10, 1});
    chassis.waitUntilDone();
    ziga.set_value(0);
    chassis.turnToHeading(0, 800, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    chassis.moveToPoint(25, -2, 1500, {true, 70, 10, 1});
}