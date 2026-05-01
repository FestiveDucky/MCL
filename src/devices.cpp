#include "main.h"
#include "devices.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include <cmath>
#include <limits>

// namespace {
// lemlib::MCLSettings makeMCLSettings() {
//     lemlib::MCLSettings cfg(500); // Number of particles tracked by MCL.

//     cfg.distanceSensors = {
//         {14, {-4.75f, 7.0f, 0.0f}},            // Front sensor (old behavior): port 7.
//         {4, {-4.75f, 1.3f, -1.57079632679f}}, // Left sensor (old behavior): port 5.
//         {20, {5.0f, 2.75f, 1.57079632679f}},   // Right sensor (old behavior): port 6.
//     };

//     cfg.sigma0XY = 0.05f;         // Baseline XY process noise each cycle (in).
//     cfg.kDistXY = 0.50f;          // Extra XY noise per inch translated.
//     cfg.kTurnXY = 0.20f;          // Extra XY noise per radian turned.
//     cfg.maxStartPosErrorIn = 2.0f; // Initial particle spread radius (in).
//     cfg.neffResampleThreshold = 0.50f; // Resample when Neff drops below this fraction of particle count.

//     cfg.clampDeltaSForNoise = true; // Caps translation before noise scaling.
//     cfg.maxDeltaSForNoise = 3.0f;   // Max translation used in noise model (in).
//     cfg.clampSigmaXY = true;        // Caps computed sigmaXY to prevent blowups.
//     cfg.maxSigmaXY = 1.50f;         // Upper bound for sigmaXY (in).

//     cfg.estMsBandwidth = 4.0f;      // Mean-shift kernel radius (in).
//     cfg.estMsIters = 6;             // Max mean-shift refinement iterations.
//     cfg.estMsEpsStop = 0.1f;        // Mean-shift convergence threshold (in).
//     cfg.estUseHuberRefinement = true; // Enables robust Huber refinement pass.
//     cfg.estUseEmaSmoothing = true;  // Enables adaptive EMA smoothing on estimated XY pose.
//     cfg.estHuberIters = 3;          // Max Huber refinement iterations.
//     cfg.estHuberGateMult = 2.0f;    // Huber neighborhood gate as bandwidth multiplier.
//     cfg.estHuberDeltaMult = 0.5f;   // Huber delta as bandwidth multiplier.
//     cfg.estAlphaMin = 0.15f;        // Min smoothing factor for final pose EMA.
//     cfg.estAlphaMax = 0.88f;        // Max smoothing factor for final pose EMA.
//     cfg.estSigmaLo = 1.5f;          // Spread where EMA starts using higher smoothing.
//     cfg.estSigmaHi = 9.0f;          // Spread where EMA reaches max smoothing.
//     cfg.estJumpThresh = 15.0f;      // Innovation threshold for jump suppression (in).
//     cfg.estAlphaJump = 0.92f;       // EMA alpha used when a jump is detected.

//     cfg.clampOobParticles = true;   // Clamps particles to field bounds if out of bounds.
//     cfg.penalizeOobParticles = true; // Downweights particles that leave bounds.
//     cfg.oobWeightMult = 1e-3;       // Weight multiplier applied to OOB particles.
//     cfg.useFieldMargin = true;      // Shrinks particle-valid region inward from walls.
//     cfg.fieldMarginIn = 5.5f;       // Margin size from field walls for particle bounds (in).
//     cfg.useSensorConfidence = true; // Uses sensor confidence to blend likelihood strength.
//     cfg.sensorConfMax = 63.0f;      // Confidence value mapped to full trust.
//     cfg.useNoHitModel = true;       // Treats >zMax as explicit "no wall hit" evidence.
//     cfg.noHitPenalty = 0.05f;      // Penalty if particle expected a wall during no-hit.

//     cfg.fieldHalf = 70.75f;         // Half field size from center to wall (in).
//     cfg.zMin = 0.1f;                // Minimum accepted distance measurement (in).
//     cfg.zMax = 85.0f;               // Maximum accepted distance measurement (in).
//     cfg.sigmaD = 8.0f;              // Sensor model triangle half-width scale (in).
//     cfg.pFloor = 1e-4;              // Minimum per-sensor likelihood floor.
//     cfg.wHit = 0.90f;               // Reserved hit-model blend weight (for future model variants).
//     cfg.wRand = 1.0f - cfg.wHit;    // Reserved random-model blend weight.

//     return cfg;
// }
// } // namespace


namespace {
lemlib::MCLSettings makeMCLSettings() {
    lemlib::MCLSettings cfg(400); // Number of particles tracked by MCL.

    cfg.distanceSensors = {
        {5, {4.f, 5.125f, 0.0f}},            // Front sensor (old behavior): port 7.
        {6, {-4.5f, 1.42f, -1.57079632679f}}, // Left sensor (old behavior): port 5.
        {14, {4.25f, -3.75f, 1.57079632679f}},   // Right sensor (old behavior): port 6.
        // {port, x offset, y offset, angle (rad)}
    };
    cfg.ignoredHitRegions = {
        // {-10, 10, -10, 10}, // middle goal


        // {xMin, xMax, yMin, yMax},
    };

    cfg.sigma0XY = 0.06f;         // Baseline XY process noise each cycle (in).
    cfg.kDistXY = 0.58f;          // Extra XY noise per inch translated.
    cfg.kTurnXY = 0.24f;          // Extra XY noise per radian turned.
    cfg.maxStartPosErrorIn = 5.0f; // Initial particle spread radius (in).
    cfg.neffResampleThreshold = 0.80f; // Resample when Neff drops below this fraction of particle count.

    cfg.clampDeltaSForNoise = true; // Caps translation before noise scaling.
    cfg.maxDeltaSForNoise = 3.0f;   // Max translation used in noise model (in).
    cfg.clampSigmaXY = true;        // Caps computed sigmaXY to prevent blowups.
    cfg.maxSigmaXY = 1.75f;         // Upper bound for sigmaXY (in).

    cfg.estMsBandwidth = 8.5f;      // Mean-shift kernel radius (in).
    cfg.estMsIters = 6;             // Max mean-shift refinement iterations.
    cfg.estMsEpsStop = 0.1f;        // Mean-shift convergence threshold (in).
    cfg.estUseHuberRefinement = true; // Enables robust Huber refinement pass.
    cfg.estUseEmaSmoothing = true;  // Enables adaptive EMA smoothing on estimated XY pose.
    cfg.estHuberIters = 3;          // Max Huber refinement iterations.
    cfg.estHuberGateMult = 2.0f;    // Huber neighborhood gate as bandwidth multiplier.
    cfg.estHuberDeltaMult = 0.5f;   // Huber delta as bandwidth multiplier.
    cfg.estAlphaMin = 0.10f;        // Min smoothing factor for final pose EMA.
    cfg.estAlphaMax = 0.80f;        // Max smoothing factor for final pose EMA.
    cfg.estSigmaLo = 2.0f;          // Spread where EMA starts using higher smoothing.
    cfg.estSigmaHi = 11.0f;         // Spread where EMA reaches max smoothing.
    cfg.estJumpThresh = 22.0f;      // Innovation threshold for jump suppression (in).
    cfg.estAlphaJump = 0.84f;       // EMA alpha used when a jump is detected.

    cfg.clampOobParticles = true;   // Clamps particles to field bounds if out of bounds.
    cfg.penalizeOobParticles = true; // Downweights particles that leave bounds.
    cfg.oobWeightMult = 1e-3;       // Weight multiplier applied to OOB particles.
    cfg.useFieldMargin = true;      // Shrinks particle-valid region inward from walls.
    cfg.fieldMarginIn = 3.f;       // Margin size from field walls for particle bounds (in).
    cfg.useSensorConfidence = true; // Uses sensor confidence to blend likelihood strength.
    cfg.sensorConfMax = 63.0f;      // Confidence value mapped to full trust.
    cfg.useNoHitModel = true;       // Treats >zMax as explicit "no wall hit" evidence.
    cfg.noHitPenalty = 0.035f;      // Penalty if particle expected a wall during no-hit.

    cfg.fieldHalf = 70.75f;         // Half field size from center to wall (in).
    cfg.zMin = 0.1f;                // Minimum accepted distance measurement (in).
    cfg.zMax = 85.0f;               // Maximum accepted distance measurement (in).
    cfg.sigmaD = 3.0f;              // Sensor model triangle half-width scale (in).
    cfg.pFloor = 5e-5;              // Minimum per-sensor likelihood floor.
    cfg.wHit = 0.90f;               // Reserved hit-model blend weight (for future model variants).
    cfg.wRand = 1.0f - cfg.wHit;    // Reserved random-model blend weight.

    return cfg;
}
} // namespace

lemlib::MCLSettings settings = makeMCLSettings();

pros::MotorGroup left_motor_group({1, -2, -15}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({17, -19, 20}, pros::MotorGears::blue);

pros::Motor bottom_intake(16, pros::MotorGears::blue);
pros::Motor top_intake(-18, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.375, // track width
                              lemlib::Omniwheel::NEW_325,
                              480, // drivetrain rpm
                              1 // horizontal drift
);

// imu
pros::Imu imu(9);
pros::Rotation vertical_rotation(7);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, -0.5);


// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1 &vertical_tracking_wheel
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
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.82, // proportional gain (kP) 1.72
                                              0, // integral gain (kI)
                                              12.5, // derivative gain (kD) // was 16
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in degrees
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(15, // joystick deadband out of 127
                                     15, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
); // 1.03

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(15, // joystick deadband out of 127
                                  15, // minimum output where drivetrain will move out of 127
                                  1.032 // expo curve gain
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

pros::adi::AnalogIn potentiometer ('F'); // Auton selector

pros::adi::Pneumatics scraper_piston = pros::adi::Pneumatics('A', false);
pros::adi::Pneumatics descore = pros::adi::Pneumatics('B', true);
pros::adi::Pneumatics middlescore_piston = pros::adi::Pneumatics('C', true);
pros::adi::Pneumatics top_score = pros::adi::Pneumatics('H', false);
pros::adi::Pneumatics intake_roller = pros::adi::Pneumatics('E', true);

namespace {
constexpr float DISTANCE_RESET_MAX_IN = 200.0f;
constexpr float DISTANCE_RESET_RAY_EPS = 1e-5f;
constexpr float DEG_TO_RAD = 0.01745329251994329577f;

struct WallHit {
    bool valid = false;
    bool solvesX = false; // true => x wall (x = +/-fieldHalf), false => y wall.
    float wallCoord = 0.0f;
};

struct AxisEstimate {
    bool hasX = false;
    bool hasY = false;
    float x = 0.0f;
    float y = 0.0f;
};

bool inRange(float value, float min, float max) {
    return value >= min && value <= max;
}

// Select the wall this ray most likely hits first, using current pose as the wall-selection seed.
WallHit raycastWall(float x0, float y0, float dx, float dy, float wallMin, float wallMax) {
    float bestT = std::numeric_limits<float>::infinity();
    WallHit hit{};

    if (std::fabs(dx) > DISTANCE_RESET_RAY_EPS) {
        float t = (wallMin - x0) / dx;
        if (t >= 0.0f) {
            const float y = y0 + t * dy;
            if (inRange(y, wallMin, wallMax) && t < bestT) {
                bestT = t;
                hit = {true, true, wallMin};
            }
        }

        t = (wallMax - x0) / dx;
        if (t >= 0.0f) {
            const float y = y0 + t * dy;
            if (inRange(y, wallMin, wallMax) && t < bestT) {
                bestT = t;
                hit = {true, true, wallMax};
            }
        }
    }

    if (std::fabs(dy) > DISTANCE_RESET_RAY_EPS) {
        float t = (wallMin - y0) / dy;
        if (t >= 0.0f) {
            const float x = x0 + t * dx;
            if (inRange(x, wallMin, wallMax) && t < bestT) {
                bestT = t;
                hit = {true, false, wallMin};
            }
        }

        t = (wallMax - y0) / dy;
        if (t >= 0.0f) {
            const float x = x0 + t * dx;
            if (inRange(x, wallMin, wallMax) && t < bestT) {
                bestT = t;
                hit = {true, false, wallMax};
            }
        }
    }

    return hit;
}

AxisEstimate estimateAxisFromSensor(std::size_t sensorIndex, const lemlib::Pose& currentPoseDeg) {
    AxisEstimate estimate{};
    const auto& cfg = lemlib::getMCLSettings();
    if (sensorIndex >= cfg.distanceSensors.size()) return estimate;

    const float readingIn = lemlib::getDistanceInchesByIndex(sensorIndex);
    if (!std::isfinite(readingIn) || readingIn <= 0.0f || readingIn > DISTANCE_RESET_MAX_IN) return estimate;

    const auto& mount = cfg.distanceSensors[sensorIndex].mount;
    const float headingRad = static_cast<float>(currentPoseDeg.theta) * DEG_TO_RAD;
    const float sh = std::sin(headingRad);
    const float ch = std::cos(headingRad);

    // Robot-frame mount offset transformed to field frame (same convention as MCL update code).
    const float mountOffsetX = ch * mount.xOffset + sh * mount.yOffset;
    const float mountOffsetY = -sh * mount.xOffset + ch * mount.yOffset;

    const float sensorX = static_cast<float>(currentPoseDeg.x) + mountOffsetX;
    const float sensorY = static_cast<float>(currentPoseDeg.y) + mountOffsetY;

    const float rayAngle = headingRad + mount.headingOffset;
    const float rayDirX = std::sin(rayAngle);
    const float rayDirY = std::cos(rayAngle);

    const WallHit hit = raycastWall(sensorX, sensorY, rayDirX, rayDirY, -cfg.fieldHalf, cfg.fieldHalf);
    if (!hit.valid) return estimate;

    // If we hit a vertical wall, solve x. If horizontal wall, solve y.
    if (hit.solvesX) {
        estimate.hasX = true;
        estimate.x = hit.wallCoord - (mountOffsetX + readingIn * rayDirX);
    } else {
        estimate.hasY = true;
        estimate.y = hit.wallCoord - (mountOffsetY + readingIn * rayDirY);
    }

    return estimate;
}
} // namespace

bool resetPositionFromDistanceSensor(std::size_t sensorIndex) {
    const lemlib::Pose currentPoseDeg = chassis.getPose(false);
    const AxisEstimate estimate = estimateAxisFromSensor(sensorIndex, currentPoseDeg);
    if (!estimate.hasX && !estimate.hasY) return false;

    const float newX = estimate.hasX ? estimate.x : currentPoseDeg.x;
    const float newY = estimate.hasY ? estimate.y : currentPoseDeg.y;

    // Use setPose so odom pose and MCL particles are re-seeded around this trusted reset point.
    chassis.setPose(newX, newY, currentPoseDeg.theta, false);
    return true;
}

bool resetPositionFromTwoDistanceSensors(std::size_t sensorA, std::size_t sensorB) {
    if (sensorA == sensorB) return resetPositionFromDistanceSensor(sensorA);

    const lemlib::Pose currentPoseDeg = chassis.getPose(false);
    const AxisEstimate estimateA = estimateAxisFromSensor(sensorA, currentPoseDeg);
    const AxisEstimate estimateB = estimateAxisFromSensor(sensorB, currentPoseDeg);

    double sumX = 0.0;
    double sumY = 0.0;
    int xCount = 0;
    int yCount = 0;

    if (estimateA.hasX) {
        sumX += estimateA.x;
        xCount++;
    }
    if (estimateA.hasY) {
        sumY += estimateA.y;
        yCount++;
    }
    if (estimateB.hasX) {
        sumX += estimateB.x;
        xCount++;
    }
    if (estimateB.hasY) {
        sumY += estimateB.y;
        yCount++;
    }

    if (xCount == 0 && yCount == 0) return false;

    const float newX = (xCount > 0) ? static_cast<float>(sumX / xCount) : currentPoseDeg.x;
    const float newY = (yCount > 0) ? static_cast<float>(sumY / yCount) : currentPoseDeg.y;

    chassis.setPose(newX, newY, currentPoseDeg.theta, false);
    return true;
}

bool resetPositionFromDistanceSensors() {
    const auto& cfg = lemlib::getMCLSettings();
    if (cfg.distanceSensors.empty()) return false;

    const lemlib::Pose currentPoseDeg = chassis.getPose(false);
    double sumX = 0.0;
    double sumY = 0.0;
    int xCount = 0;
    int yCount = 0;

    for (std::size_t i = 0; i < cfg.distanceSensors.size(); i++) {
        const AxisEstimate estimate = estimateAxisFromSensor(i, currentPoseDeg);
        if (estimate.hasX) {
            sumX += estimate.x;
            xCount++;
        }
        if (estimate.hasY) {
            sumY += estimate.y;
            yCount++;
        }
    }

    if (xCount == 0 && yCount == 0) return false;

    const float newX = (xCount > 0) ? static_cast<float>(sumX / xCount) : currentPoseDeg.x;
    const float newY = (yCount > 0) ? static_cast<float>(sumY / yCount) : currentPoseDeg.y;

    chassis.setPose(newX, newY, currentPoseDeg.theta, false);
    return true;
}
