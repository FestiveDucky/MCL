#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

extern lemlib::MCLSettings settings;

extern pros::Motor bottom_intake;
extern pros::Motor top_intake;

extern lemlib::Chassis chassis;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern pros::Controller controller;

extern pros::adi::AnalogIn potentiometer;

extern pros::Rotation vertical_rotation;

extern pros::adi::Pneumatics scraper_piston;
extern pros::adi::Pneumatics descore;
extern pros::adi::Pneumatics middlescore_piston;
extern pros::adi::Pneumatics top_score;
