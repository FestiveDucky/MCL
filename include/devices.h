#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include <cstddef>

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
extern pros::adi::Pneumatics intake_roller;

/**
 * Reset odometry pose from all configured MCL distance sensors.
 * Uses current heading and sensor mounts, then re-centers particles at the solved pose.
 * Returns true if at least one axis (x or y) was solved and applied.
 */
bool resetPositionFromDistanceSensors();

/**
 * Reset odometry pose from one configured MCL distance sensor by index.
 * Returns true if the selected sensor produced a valid axis reset.
 */
bool resetPositionFromDistanceSensor(std::size_t sensorIndex);

/**
 * Reset odometry pose using exactly two selected distance sensors.
 * Each valid sensor contributes an x or y estimate; matching axes are averaged.
 * Returns true if at least one axis (x or y) was solved and applied.
 */
bool resetPositionFromTwoDistanceSensors(std::size_t sensorA, std::size_t sensorB);
