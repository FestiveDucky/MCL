#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"

namespace {
constexpr float NOMINAL_LOOP_DT = 0.01f;
constexpr float MIN_LOOP_DT = 0.005f;
constexpr float MAX_LOOP_DT = 0.03f;
constexpr float EPS = 1e-4f;

#ifndef LEMLIB_RAMSETE_PRINTF_DEBUG
#define LEMLIB_RAMSETE_PRINTF_DEBUG 1
#endif

#ifndef LEMLIB_RAMSETE_PRINTF_DECIMATE
#define LEMLIB_RAMSETE_PRINTF_DECIMATE 1
#endif

float clamp01(float value) { return std::clamp(value, 0.0f, 1.0f); }

float wrapAngle(float angle) { return std::remainder(angle, 2.0f * static_cast<float>(M_PI)); }

float sinc(float angle) {
    if (std::fabs(angle) < 1e-3f) return 1.0f - (angle * angle) / 6.0f;
    return std::sin(angle) / angle;
}

float smoothstep(float x, float edge0, float edge1) {
    if (edge1 <= edge0) return x >= edge1 ? 1.0f : 0.0f;
    const float t = clamp01((x - edge0) / (edge1 - edge0));
    return t * t * (3.0f - (2.0f * t));
}

float angleLerp(float start, float end, float t) { return wrapAngle(start + lemlib::angleError(end, start, true) * t); }

float angleEma(float current, float previous, float alpha) {
    return wrapAngle(previous + lemlib::angleError(current, previous, true) * alpha);
}

float signOrZero(float value) {
    if (std::fabs(value) < EPS) return 0.0f;
    return value > 0.0f ? 1.0f : -1.0f;
}

float average(const std::vector<double>& values) {
    if (values.empty()) return 0.0f;
    double sum = 0.0;
    for (double value : values) sum += value;
    return static_cast<float>(sum / values.size());
}

enum class RamsetePhase { Translate, FinalTurn };
} // namespace

void lemlib::Chassis::moveToPointRamsete(float x, float y, float theta, int timeout, MoveToPointRamseteParams params,
                                         bool async) {
    params.maxSpeed = std::clamp(std::fabs(params.maxSpeed), 0.0f, 127.0f);
    params.maxAccel = std::fabs(params.maxAccel);
    params.b = std::max(std::fabs(params.b), EPS);
    params.zeta = std::max(std::fabs(params.zeta), EPS);
    params.reverseEnableDist = std::fabs(params.reverseEnableDist);
    params.maxReverseSpeed = std::fabs(params.maxReverseSpeed);
    params.turnPhaseEnterDist = std::fabs(params.turnPhaseEnterDist);
    params.turnPhaseEnterSpeed = std::fabs(params.turnPhaseEnterSpeed);
    params.finalHoldMaxSpeed = std::fabs(params.finalHoldMaxSpeed);
    params.maxOmegaTranslate = std::max(std::fabs(params.maxOmegaTranslate), EPS);
    params.maxOmegaFinal = std::max(std::fabs(params.maxOmegaFinal), EPS);
    params.finalHeadingK = std::max(std::fabs(params.finalHeadingK), EPS);
    params.headingBlendStart = std::clamp(params.headingBlendStart, 0.0f, 1.0f);
    params.poseFilterAlpha = std::clamp(params.poseFilterAlpha, 0.0f, 1.0f);
    params.lateralDeadband = std::fabs(params.lateralDeadband);
    params.maxVoltage = std::clamp(std::fabs(params.maxVoltage), 0.0f, 12000.0f);
    params.voltageSlew = std::fabs(params.voltageSlew);

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([this, x, y, theta, timeout, params]() {
            moveToPointRamsete(x, y, theta, timeout, params, false);
        });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    Pose pose = getPose(true, true);
    Pose filteredPose = pose;
    Pose lastPose = pose;
    distTraveled = 0;
    Timer timer(timeout);

    const Pose start = pose;
    const Pose goal(x, y, 0);
    const float finalTheta = M_PI_2 - degToRad(theta); // convert from lemlib heading convention to standard frame
    const Pose delta = goal - start;
    const float totalDist = std::hypot(delta.x, delta.y);
    const float invTotalDist = totalDist > EPS ? 1.0f / totalDist : 0.0f;
    const float pathUnitX = delta.x * invTotalDist;
    const float pathUnitY = delta.y * invTotalDist;

    float pathHeading = totalDist > EPS ? std::atan2(delta.y, delta.x) : start.theta;
    if (!params.forwards) pathHeading = wrapAngle(pathHeading + static_cast<float>(M_PI));

    const float wheelCircumference = drivetrain.wheelDiameter * static_cast<float>(M_PI);
    const float maxWheelSpeed = wheelCircumference * drivetrain.rpm / 60.0f; // in/s
    const float maxLinearSpeed = maxWheelSpeed * (params.maxSpeed / 127.0f);
    const float maxAccel = params.maxAccel > EPS ? params.maxAccel : maxLinearSpeed / 0.25f;
    const float allowedSurfaceSpeed = maxLinearSpeed;

    float voltageSlew = params.voltageSlew;
    if (voltageSlew == 0 && lateralSettings.slew > 0) voltageSlew = params.maxVoltage * (lateralSettings.slew / 127.0f);

    float kV = params.kV;
    if (kV == 0 && maxWheelSpeed > EPS) kV = params.maxVoltage / maxWheelSpeed;

    float profileS = 0.0f; // profile progress along path
    float profileV = 0.0f; // profile speed magnitude
    float prevRefHeading = pathHeading;
    float prevLeftTarget = 0.0f;
    float prevRightTarget = 0.0f;
    float prevLeftVoltage = 0.0f;
    float prevRightVoltage = 0.0f;
    float lastDistError = totalDist;
    float lastHeadingError = 0.0f;
    RamsetePhase phase = RamsetePhase::Translate;
#if LEMLIB_RAMSETE_PRINTF_DEBUG
    int loopCounter = 0;
#endif
    const float kSVelocityDeadband = std::max(0.25f, allowedSurfaceSpeed * 0.02f);
    const float maxProfileLead = std::max(5.0f, allowedSurfaceSpeed * 0.15f);
    std::uint32_t lastLoopTime = pros::micros();

    while (!timer.isDone() &&
           !(lateralSmallExit.getExit() && lateralLargeExit.getExit() && angularSmallExit.getExit() &&
             angularLargeExit.getExit()) &&
           this->motionRunning) {
        const std::uint32_t now = pros::micros();
        float loopDt = static_cast<float>(now - lastLoopTime) / 1e6f;
        lastLoopTime = now;
        if (loopDt <= 0.0f) loopDt = NOMINAL_LOOP_DT;
        loopDt = std::clamp(loopDt, MIN_LOOP_DT, MAX_LOOP_DT);

        pose = getPose(true, true);

        // update distance traveled for waitUntil
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // slight filtering to reject jitter while preserving responsiveness
        if (params.poseFilterAlpha <= EPS) filteredPose = pose;
        else {
            // preserve approximately the same smoothing profile as nominal 10ms loops
            const float poseAlpha =
                1.0f - std::pow(1.0f - params.poseFilterAlpha, std::max(loopDt, EPS) / NOMINAL_LOOP_DT);
            filteredPose.x = ema(pose.x, filteredPose.x, poseAlpha);
            filteredPose.y = ema(pose.y, filteredPose.y, poseAlpha);
            filteredPose.theta = angleEma(pose.theta, filteredPose.theta, poseAlpha);
        }

        const float distToGoal = filteredPose.distance(goal);

        // trapezoidal profile (online): cap speed by stop distance and acceleration
        float profileTargetV = 0.0f;
        float integratedProfileS = profileS;
        float maxProfileS = totalDist;
        bool leadCapActive = false;
        if (phase == RamsetePhase::Translate) {
            const float remaining = std::max(0.0f, totalDist - profileS);
            const float stopLimitedV = maxAccel > EPS ? std::sqrt(2.0f * maxAccel * remaining) : maxLinearSpeed;
            profileTargetV = std::min(maxLinearSpeed, stopLimitedV);
            profileV = slew(profileTargetV, profileV, maxAccel * loopDt);
            if (remaining <= EPS) profileV = 0.0f;

            const float prevProfileS = profileS;
            integratedProfileS = std::clamp(profileS + profileV * loopDt, 0.0f, totalDist);
            if (totalDist > EPS) {
                const float alongTrack = (filteredPose.x - start.x) * pathUnitX + (filteredPose.y - start.y) * pathUnitY;
                maxProfileS = std::clamp(alongTrack + maxProfileLead, 0.0f, totalDist);
                profileS = std::clamp(integratedProfileS, prevProfileS, std::max(prevProfileS, maxProfileS));
                leadCapActive = profileS + EPS < integratedProfileS;
            } else profileS = integratedProfileS;
        } else {
            profileS = totalDist;
            profileV = 0.0f;
            integratedProfileS = totalDist;
            maxProfileS = totalDist;
        }

        if (phase == RamsetePhase::Translate &&
            ((distToGoal <= params.turnPhaseEnterDist && std::fabs(profileV) <= params.turnPhaseEnterSpeed) ||
             profileS >= std::max(0.0f, totalDist - 0.1f))) {
            phase = RamsetePhase::FinalTurn;
            profileS = totalDist;
            profileV = 0.0f;
            profileTargetV = 0.0f;
            integratedProfileS = totalDist;
            maxProfileS = totalDist;
            prevRefHeading = finalTheta;
            leadCapActive = false;
        }

        float refX = goal.x;
        float refY = goal.y;
        float refHeading = finalTheta;
        float refV = 0.0f;
        float refOmega = 0.0f;
        if (phase == RamsetePhase::Translate) {
            refX = totalDist > EPS ? start.x + pathUnitX * profileS : goal.x;
            refY = totalDist > EPS ? start.y + pathUnitY * profileS : goal.y;
            if (params.pointFirstThenTurn) refHeading = pathHeading;
            else {
                const float progress = totalDist > EPS ? profileS / totalDist : 1.0f;
                const float blend = smoothstep(progress, params.headingBlendStart, 1.0f);
                refHeading = angleLerp(pathHeading, finalTheta, blend);
            }
            refOmega = angleError(refHeading, prevRefHeading, true) / loopDt;
            prevRefHeading = refHeading;
            // Keep heading flipped and velocity signed for reverse moves so the
            // reference kinematics (xDot/yDot vs heading) stay consistent.
            refV = params.forwards ? profileV : -profileV;
            if (drivetrain.trackWidth > EPS) {
                const float maxRefOmega =
                    std::max(0.0f, (allowedSurfaceSpeed - std::fabs(refV)) * 2.0f / drivetrain.trackWidth);
                refOmega = std::clamp(refOmega, -maxRefOmega, maxRefOmega);
            }
        } else {
            refOmega = std::clamp(params.finalHeadingK * angleError(finalTheta, filteredPose.theta, true),
                                  -params.maxOmegaFinal, params.maxOmegaFinal);
            prevRefHeading = refHeading;
        }

        // pose error in robot frame
        const float dx = refX - filteredPose.x;
        const float dy = refY - filteredPose.y;
        const float cosTheta = std::cos(filteredPose.theta);
        const float sinTheta = std::sin(filteredPose.theta);
        const float ex = cosTheta * dx + sinTheta * dy;
        float ey = -sinTheta * dx + cosTheta * dy;
        if (std::fabs(ey) < params.lateralDeadband) ey = 0.0f;
        const float eTheta = angleError(refHeading, filteredPose.theta, true);

        // RAMSETE chassis command
        const float gain = 2.0f * params.zeta * std::sqrt((refOmega * refOmega) + (params.b * refV * refV));
        float vCmd = refV * std::cos(eTheta) + gain * ex;
        float omegaCmd = refOmega + gain * eTheta + (params.b * refV * sinc(eTheta) * ey);
        bool dirClampActive = false;
        bool revClampActive = false;
        if (phase == RamsetePhase::Translate) {
            if (params.forwards) {
                if (params.allowReverseNearGoal && distToGoal <= params.reverseEnableDist) {
                    if (vCmd < -params.maxReverseSpeed) {
                        vCmd = -params.maxReverseSpeed;
                        revClampActive = true;
                    }
                } else if (vCmd < 0.0f) {
                    vCmd = 0.0f;
                    dirClampActive = true;
                }
            } else {
                if (params.allowReverseNearGoal && distToGoal <= params.reverseEnableDist) {
                    if (vCmd > params.maxReverseSpeed) {
                        vCmd = params.maxReverseSpeed;
                        revClampActive = true;
                    }
                } else if (vCmd > 0.0f) {
                    vCmd = 0.0f;
                    dirClampActive = true;
                }
            }
        } else vCmd = std::clamp(vCmd, -params.finalHoldMaxSpeed, params.finalHoldMaxSpeed);

        const float omegaLimit = phase == RamsetePhase::Translate ? params.maxOmegaTranslate : params.maxOmegaFinal;
        omegaCmd = std::clamp(omegaCmd, -omegaLimit, omegaLimit);

        // chassis -> wheel velocity targets (in/s)
        float leftTarget = vCmd - omegaCmd * drivetrain.trackWidth / 2.0f;
        float rightTarget = vCmd + omegaCmd * drivetrain.trackWidth / 2.0f;
        const float wheelScale =
            std::max(std::fabs(leftTarget), std::fabs(rightTarget)) / std::max(allowedSurfaceSpeed, EPS);
        if (wheelScale > 1.0f) {
            leftTarget /= wheelScale;
            rightTarget /= wheelScale;
        }

        const float leftTargetAccel = (leftTarget - prevLeftTarget) / loopDt;
        const float rightTargetAccel = (rightTarget - prevRightTarget) / loopDt;
        prevLeftTarget = leftTarget;
        prevRightTarget = rightTarget;

        // use motor encoders for wheel-speed feedback
        const float leftMeasured = average(drivetrain.leftMotors->get_actual_velocity_all()) * wheelCircumference / 60.0f;
        const float rightMeasured =
            average(drivetrain.rightMotors->get_actual_velocity_all()) * wheelCircumference / 60.0f;

        // wheel velocity targets -> motor voltages
        const float leftStatic = std::fabs(leftTarget) > kSVelocityDeadband ? params.kS * signOrZero(leftTarget) : 0.0f;
        const float rightStatic =
            std::fabs(rightTarget) > kSVelocityDeadband ? params.kS * signOrZero(rightTarget) : 0.0f;
        float leftVoltage = leftStatic + kV * leftTarget + params.kA * leftTargetAccel +
                            params.kP * (leftTarget - leftMeasured);
        float rightVoltage = rightStatic + kV * rightTarget + params.kA * rightTargetAccel +
                             params.kP * (rightTarget - rightMeasured);

        leftVoltage = std::clamp(leftVoltage, -params.maxVoltage, params.maxVoltage);
        rightVoltage = std::clamp(rightVoltage, -params.maxVoltage, params.maxVoltage);

        if (voltageSlew > 0.0f) {
            const float voltageMaxDelta = voltageSlew * (loopDt / NOMINAL_LOOP_DT);
            leftVoltage = slew(leftVoltage, prevLeftVoltage, voltageMaxDelta);
            rightVoltage = slew(rightVoltage, prevRightVoltage, voltageMaxDelta);
        }
        prevLeftVoltage = leftVoltage;
        prevRightVoltage = rightVoltage;

        drivetrain.leftMotors->move_voltage(static_cast<std::int32_t>(std::lround(leftVoltage)));
        drivetrain.rightMotors->move_voltage(static_cast<std::int32_t>(std::lround(rightVoltage)));

        // settle on final pose (point + heading)
        const float distError = filteredPose.distance(goal);
        const float headingError = radToDeg(angleError(finalTheta, filteredPose.theta, true));
        lastDistError = distError;
        lastHeadingError = headingError;
        lateralSmallExit.update(distError);
        lateralLargeExit.update(distError);
        angularSmallExit.update(headingError);
        angularLargeExit.update(headingError);

        infoSink()->debug("moveToPointRamsete V(L,R): ({}, {}), err(d,a): ({}, {})", leftVoltage, rightVoltage, distError,
                          headingError);

#if LEMLIB_RAMSETE_PRINTF_DEBUG
        if ((loopCounter++ % LEMLIB_RAMSETE_PRINTF_DECIMATE) == 0) {
            const char phaseChar = phase == RamsetePhase::Translate ? 'T' : 'F';
            std::printf("[RAMSETE] t=%lu dt=%.3f phase=%c vRefTarget=%.2f vRef=%.2f s=%.2f sInt=%.2f sMax=%.2f "
                        "leadCap=%d dirClamp=%d revClamp=%d vCmd=%.2f wCmd=%.2f lT=%.2f rT=%.2f lV=%.0f rV=%.0f "
                        "errD=%.2f errH=%.2f\n",
                        static_cast<unsigned long>(pros::millis()), loopDt, phaseChar, profileTargetV, profileV,
                        profileS, integratedProfileS, maxProfileS, static_cast<int>(leadCapActive),
                        static_cast<int>(dirClampActive), static_cast<int>(revClampActive), vCmd, omegaCmd, leftTarget,
                        rightTarget, leftVoltage, rightVoltage, distError, headingError);
        }
#endif

        pros::delay(10);
    }

#if LEMLIB_RAMSETE_PRINTF_DEBUG
    std::printf("[RAMSETE-END] t=%lu timeout=%d settled=%d cancelled=%d finalErrD=%.2f finalErrH=%.2f\n",
                static_cast<unsigned long>(pros::millis()), static_cast<int>(timer.isDone()),
                static_cast<int>(lateralSmallExit.getExit() && lateralLargeExit.getExit() && angularSmallExit.getExit() &&
                                 angularLargeExit.getExit()),
                static_cast<int>(!this->motionRunning), lastDistError, lastHeadingError);
#endif

    // stop the drivetrain
    drivetrain.leftMotors->move_voltage(0);
    drivetrain.rightMotors->move_voltage(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}
