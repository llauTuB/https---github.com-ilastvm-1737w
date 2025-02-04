#include "main.h" // IWYU pragma: keep

lemlib::PID arm_lift(
   0.87,
   0,
   0.2,
   0,
   false 
);


bool liftPidActive = false;
double liftTarget;

void moveArmToPosition(double target) {
    liftPidActive = true;
    liftTarget = target;
}

void armPID(){
        while (true) {
            if (liftPidActive) {
                
                float power = arm_lift.update(liftTarget - rotation.get_position()/100.0);
                power = fmin(fmax(power, -127), 127);
                arm.move(power);

                
                if (std::abs(liftTarget - arm.get_position()) < 0.8) {
                    arm.move_voltage(0);
                    liftPidActive = false;
                }
            } else {
                pros::delay(20); 
            }
        }
    };


void opcontrolLift() {


    
        if (!liftPidActive) {
            arm.move_voltage((master.get_digital(pros::E_CONTROLLER_DIGITAL_B) - master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) * 12000);
        } else {
            arm.move_voltage(0);  
        }
        
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            moveArmToPosition(MID);
            
        }
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            moveArmToPosition(UP);  
            
        } 

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            moveArmToPosition(DOWN);    
        } 
}
