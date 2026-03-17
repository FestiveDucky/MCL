#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

extern lemlib::MCLSettings settings;

extern pros::Motor bottom_intake;
extern pros::Motor top_intake;
extern pros::Motor mid_intake;

extern lemlib::Chassis chassis;
extern pros::Imu imu;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern pros::Controller controller;

extern pros::adi::AnalogIn potentiometer;

extern pros::Rotation vertical_rotation;
extern pros::Rotation horizontal_rotation;

extern pros::adi::DigitalOut scraper;
extern pros::adi::DigitalOut wing;
extern pros::adi::DigitalOut horLift;
extern pros::adi::DigitalOut verLift;
extern pros::adi::DigitalOut flappier;
extern pros::adi::DigitalOut flappy;
extern pros::adi::DigitalOut descore;

// Reset chassis X or Y using MCL distance sensors (use only when perpendicular to wall)
void resetPositionFront();
void resetPositionBack();
void resetPositionLeft();
void resetPositionRight();

// Move until a distance sensor reading is within [targetInches - toleranceInches, targetInches + toleranceInches].
// Sensor: 0=front, 1=left, 2=right, 3=back. timeoutMs: 0 = use 60s safety cap; otherwise stop after this many ms.
void moveUntilDistance(int sensorIndex, float targetInches, float toleranceInches, double power, double turn, unsigned int timeoutMs = 5000);
