#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;

extern lemlib::Drivetrain drivetrain;

extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;

extern lemlib::OdomSensors sensors;

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

extern lemlib::Chassis chassis;