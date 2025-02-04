#pragma once
#include "lemlib/pid.hpp" 


extern lemlib::PID arm_Lift;

extern bool liftPidActive;
extern double liftTarget;

enum lift_state {
  DOWN = 0,
  MID = 50,
  UP = 290
};

void moveArmToPosition(double target);

void armPID();

void opcontrolLift();