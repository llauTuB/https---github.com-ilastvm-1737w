#include "main.h" // IWYU pragma: keep

bool mogoState = false; 
bool pisunState = false;
bool hangState = false;
bool colorState = false;

void mogo_toggle() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        mogoState = !mogoState;  
        mogo.set_value(mogoState);  
    }
}

void pisun_toggle() {
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        pisunState = !pisunState;
        ziga.set_value(pisunState);
    }
}

void hang_toggle() {
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        hangState = !hangState;
        hang.set_value(hangState);
    }
}

void color_toggle() {
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        colorState = !colorState;
        color_sort.set_value(colorState);
    }    
}