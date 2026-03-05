/**
 * Distance sensor position reset for autons.
 * Uses MCL's distance sensors and their mounts (single source of truth). Only resets X or Y based on which wall the robot is facing.
 * Use only when perpendicular to the wall.
 * Sensor index order matches MCL config in devices.cpp: 0=front, 1=left, 2=right, 3=back.
 */
#include "devices.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include <cmath>

namespace {
constexpr double MAX_VALID_DIST_IN = 200.0;

// Distance from robot center to sensor along the sensor's measurement direction (inches).
// Uses MCL DistanceSensorMount: direction is (sin(headingOffset), cos(headingOffset)), projection = xOffset*sin + yOffset*cos.
double getSensorOffsetAlongAxis(std::size_t index) {
    const lemlib::MCLSettings& cfg = lemlib::getMCLSettings();
    if (index >= cfg.distanceSensors.size()) return 0.0;
    const auto& m = cfg.distanceSensors[index].mount;
    double s = std::sin(m.headingOffset);
    double c = std::cos(m.headingOffset);
    return m.xOffset * s + m.yOffset * c;
}

void resetPositionWithSensor(std::size_t sensor_index, double sensor_angle_offset_deg) {
    double field_half_size = static_cast<double>(settings.fieldHalf);
    float sensor_reading_in = lemlib::getDistanceInchesByIndex(sensor_index);
    if (sensor_index >= 4 || !std::isfinite(sensor_reading_in) || sensor_reading_in < 0 || sensor_reading_in > MAX_VALID_DIST_IN)
        return;
    double sensor_offset = getSensorOffsetAlongAxis(sensor_index);

    lemlib::Pose cur = chassis.getPose(false);
    double robot_heading_deg = cur.theta + sensor_angle_offset_deg;
    int heading_deg = (static_cast<int>(std::round(robot_heading_deg)) + 360) % 360;

    bool reset_x = false;
    double wall_sign = 1.0;
    if (315 <= heading_deg || heading_deg <= 45) {
        reset_x = false;
        wall_sign = 1.0;
    } else if (45 < heading_deg && heading_deg <= 135) {
        reset_x = true;
        wall_sign = 1.0;
    } else if (135 < heading_deg && heading_deg <= 225) {
        reset_x = false;
        wall_sign = -1.0;
    } else {
        reset_x = true;
        wall_sign = -1.0;
    }

    double wall_to_center = sensor_reading_in + sensor_offset;
    double actual_pos = wall_sign * (field_half_size - wall_to_center);

    if (reset_x)
        chassis.setPose(static_cast<float>(actual_pos), cur.y, cur.theta, false);
    else
        chassis.setPose(cur.x, static_cast<float>(actual_pos), cur.theta, false);
}
} // namespace

void resetPositionFront() {
    resetPositionWithSensor(0, 0.0);
}

void resetPositionBack() {
    resetPositionWithSensor(3, 180.0);
}

void resetPositionLeft() {
    resetPositionWithSensor(1, 270.0);
}

void resetPositionRight() {
    resetPositionWithSensor(2, 90.0);
}

void moveUntilDistance(int sensor_index, float target_inches, float tolerance_inches, double power, double turn,
                      unsigned int timeout_ms) {
    if (sensor_index < 0 || sensor_index > 3) return;
    chassis.cancelAllMotions();

    int left = static_cast<int>(power + turn);
    int right = static_cast<int>(power - turn);
    left_motor_group.move(left);
    right_motor_group.move(right);

    const float min_inches = target_inches - tolerance_inches;
    const float max_inches = target_inches + tolerance_inches;
    const unsigned int limit = (timeout_ms == 0) ? 60000u : timeout_ms;
    const unsigned int start = pros::millis();

    while (pros::millis() - start < limit) {
        float reading = lemlib::getDistanceInchesByIndex(static_cast<std::size_t>(sensor_index));
        if (std::isfinite(reading) && reading >= min_inches && reading <= max_inches)
            break;
        pros::delay(20);
    }
    left_motor_group.brake();
    right_motor_group.brake();
}
