#include "main.h"
#include "lemlib/chassis/chassis.hpp"

namespace {
lemlib::MCLSettings makeMCLSettings() {
    lemlib::MCLSettings cfg(500); // Number of particles tracked by MCL.

    cfg.distanceSensors = {
        {14, {-4.75f, 7.0f, 0.0f}},            // Front sensor (old behavior): port 7.
        {4, {-4.75f, 1.3f, -1.57079632679f}}, // Left sensor (old behavior): port 5.
        {20, {5.0f, 2.75f, 1.57079632679f}},   // Right sensor (old behavior): port 6.
    };

    cfg.sigma0XY = 0.05f;         // Baseline XY process noise each cycle (in).
    cfg.kDistXY = 0.50f;          // Extra XY noise per inch translated.
    cfg.kTurnXY = 0.20f;          // Extra XY noise per radian turned.
    cfg.maxStartPosErrorIn = 2.0f; // Initial particle spread radius (in).

    cfg.clampDeltaSForNoise = true; // Caps translation before noise scaling.
    cfg.maxDeltaSForNoise = 3.0f;   // Max translation used in noise model (in).
    cfg.clampSigmaXY = true;        // Caps computed sigmaXY to prevent blowups.
    cfg.maxSigmaXY = 1.50f;         // Upper bound for sigmaXY (in).

    cfg.estMsBandwidth = 4.0f;      // Mean-shift kernel radius (in).
    cfg.estMsIters = 6;             // Max mean-shift refinement iterations.
    cfg.estMsEpsStop = 0.1f;        // Mean-shift convergence threshold (in).
    cfg.estUseHuberRefinement = true; // Enables robust Huber refinement pass.
    cfg.estHuberIters = 3;          // Max Huber refinement iterations.
    cfg.estHuberGateMult = 2.0f;    // Huber neighborhood gate as bandwidth multiplier.
    cfg.estHuberDeltaMult = 0.5f;   // Huber delta as bandwidth multiplier.
    cfg.estAlphaMin = 0.15f;        // Min smoothing factor for final pose EMA.
    cfg.estAlphaMax = 0.88f;        // Max smoothing factor for final pose EMA.
    cfg.estSigmaLo = 1.5f;          // Spread where EMA starts using higher smoothing.
    cfg.estSigmaHi = 9.0f;          // Spread where EMA reaches max smoothing.
    cfg.estJumpThresh = 15.0f;      // Innovation threshold for jump suppression (in).
    cfg.estAlphaJump = 0.92f;       // EMA alpha used when a jump is detected.

    cfg.clampOobParticles = true;   // Clamps particles to field bounds if out of bounds.
    cfg.penalizeOobParticles = true; // Downweights particles that leave bounds.
    cfg.oobWeightMult = 1e-3;       // Weight multiplier applied to OOB particles.
    cfg.useFieldMargin = true;      // Shrinks particle-valid region inward from walls.
    cfg.fieldMarginIn = 5.5f;       // Margin size from field walls for particle bounds (in).
    cfg.useSensorConfidence = true; // Uses sensor confidence to blend likelihood strength.
    cfg.sensorConfMax = 63.0f;      // Confidence value mapped to full trust.
    cfg.useNoHitModel = true;       // Treats >zMax as explicit "no wall hit" evidence.
    cfg.noHitPenalty = 0.05f;       // Penalty if particle expected a wall during no-hit.

    cfg.fieldHalf = 70.75f;         // Half field size from center to wall (in).
    cfg.zMin = 0.1f;                // Minimum accepted distance measurement (in).
    cfg.zMax = 85.0f;               // Maximum accepted distance measurement (in).
    cfg.sigmaD = 10.0f;             // Sensor model triangle half-width scale (in).
    cfg.pFloor = 1e-4;              // Minimum per-sensor likelihood floor.
    cfg.wHit = 0.90f;               // Reserved hit-model blend weight (for future model variants).
    cfg.wRand = 1.0f - cfg.wHit;    // Reserved random-model blend weight.

    return cfg;
}
} // namespace

lemlib::MCLSettings settings = makeMCLSettings();

pros::MotorGroup left_motor_group({11, -15, -13}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({13, -17, 19}, pros::MotorGears::blue);

pros::Motor bottom_intake(-10, pros::MotorGears::blue);
pros::Motor top_intake(2, pros::MotorGears::blue);

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
pros::Rotation vertical_rotation(-12);
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
                                              0.5, // small error range, in inches
                                              150, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
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

pros::adi::Pneumatics scraper_piston = pros::adi::Pneumatics('H', false);
pros::adi::Pneumatics descore = pros::adi::Pneumatics('F', false);
pros::adi::Pneumatics middlescore_piston = pros::adi::Pneumatics('G', false);
pros::adi::Pneumatics top_score = pros::adi::Pneumatics('E', true);
