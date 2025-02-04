#include "config/robot_config.hpp"
#include "main.h" // IWYU pragma: keep
#include "pros/misc.h"

bool isColorActive = true;


void color_sorter_red()
{
    while (isColorActive) {
        
        double hue = optical.get_hue();
        double prox = optical.get_proximity();

        if ((hue >= minR && hue <= maxR) && prox > 250) {
            pros::delay(30); 
            intake.move(-127);
            pros::delay(100);
            intake.move(127);
        } 
        else {
            intake.move(127);
        }

        pros::delay(60); 
    }
}

void color_sorter_blue() {
    while (isColorActive) {
        
        double hue = optical.get_hue();
        double prox = optical.get_proximity();

        if ((hue >= minB && hue <= maxB) && prox > 250) {
            pros::delay(30); 
            intake.move(-127);
            pros::delay(100);
            intake.move(127);

        } else {
            intake.move(127);
        }

        pros::delay(50); 
    }
}


void color_sort_activate() {
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        isColorActive = !isColorActive;
}