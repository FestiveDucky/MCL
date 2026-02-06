#include "main.h"
#include "lemlib/chassis/chassis.hpp"

lemlib::MCLSettings settings(500);

pros::MotorGroup left_motor_group({-11, 12, -13}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({18, -19, 20}, pros::MotorGears::blue);

// pros::Distance left_distance(0);
// double left_distance_offset = 0.0; 
// pros::Distance front_left_distance(0);
// double front_left_distance_offset = 0.0;


pros::Motor bottom_intake(-10, pros::MotorGears::blue);
pros::Motor top_intake(9, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.95, // track width
                              lemlib::Omniwheel::NEW_325,
                              480, // drivetrain rpm
                              1 // horizontal drift
);

// imu
pros::Imu imu(1);
pros::Rotation vertical_rotation(-17);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_275, 0.25);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1 &vertical_tracking_wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(3.8, // proportional gain (kP)  //was 4
                                              0, // integral gain (kI)
                                              24, // derivative gain (kD)    //was 10
                                              3, // anti windup
                                              0.1, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              15 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.82, // proportional gain (kP) 1.72
                                              0, // integral gain (kI)
                                              12.5, // derivative gain (kD) // was 16
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              200, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              15 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(15, // joystick deadband out of 127
                                     15, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
); // 1.03

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(15, // joystick deadband out of 127
                                  15, // minimum output where drivetrain will move out of 127
                                  1.005 // expo curve gain
); // 1.03

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        settings,
                        &throttle_curve, 
                        &steer_curve
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::AnalogIn potentiometer ('A'); // Auton selector

pros::adi::Pneumatics scraper_piston = pros::adi::Pneumatics('G', false);
pros::adi::Pneumatics descore = pros::adi::Pneumatics('F', false);
pros::adi::Pneumatics middlescore_piston = pros::adi::Pneumatics('H', false);
pros::adi::Pneumatics top_score = pros::adi::Pneumatics('E', true);