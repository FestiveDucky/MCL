// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include <random>
#include <cmath>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include <mutex>
#include <shared_mutex>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "particle.h"

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables
lemlib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr); // the sensors to be used for odometry
lemlib::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0); // the drivetrain to be used for odometry
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot
lemlib::MCLSettings mclSettings(0); 
lemlib::Pose oldOdomPoseCalculation(-48, -48, 0);
std::uint32_t updateStartTime = 0;
std::uint32_t updateTime = 0;
std::uint32_t prev_time = 0;
std::uint32_t frontConf = 0;
bool paused = false;
// pros::Mutex mclPoseMtx;
// pros::Mutex oldPoseMtx;

pros::Distance left_distance(5);
pros::Distance right_distance(6);
pros::Distance front_distance(7);

// Motion Noise
constexpr float SIGMA0_XY = 0.05f; // base noise (in)
constexpr float K_DIST_XY = 0.50f; // in of std dev per in traveled
constexpr float K_TURN_XY = 0.20f; // in of std dev per rad turned
constexpr float MAX_START_POS_ERROR_IN = 2.0f; // Within this error 99.7% of the time (For the normal dist setting), o.w. just within +-this range for uniform

std::vector<Particle> particles; // The possible robot poses

float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;

void lemlib::toggleMCL() {
    paused = !paused;
}

std::uint32_t lemlib::getConfidence() {
    return frontConf;
}

std::uint32_t lemlib::getCalculationTime() {
    return updateTime;
}

void lemlib::setMCLSettings(lemlib::MCLSettings settings) {
    mclSettings = settings;
}

void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

lemlib::Pose lemlib::getPose(bool radians) {
    lemlib::Pose copy = odomPose;
    if (radians) return copy;
    else return lemlib::Pose(copy.x, copy.y, radToDeg(copy.theta));
}

lemlib::Pose lemlib::getOldPose() {
    lemlib::Pose copy = oldOdomPoseCalculation;
    return copy;
}

void lemlib::setPose(lemlib::Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

lemlib::Pose lemlib::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return lemlib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return lemlib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

lemlib::Pose lemlib::estimatePose(float time, bool radians) {
    // get current position and speed
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    // calculate the change in local position
    Pose deltaLocalPose = localSpeed * time;

    // calculate the future pose
    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

void lemlib::update() {
    updateStartTime = pros::micros();

    // get the current sensor values
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    if (odomSensors.vertical1 != nullptr) vertical1Raw = odomSensors.vertical1->getDistanceTraveled();
    if (odomSensors.vertical2 != nullptr) vertical2Raw = odomSensors.vertical2->getDistanceTraveled();
    if (odomSensors.horizontal1 != nullptr) horizontal1Raw = odomSensors.horizontal1->getDistanceTraveled();
    if (odomSensors.horizontal2 != nullptr) horizontal2Raw = odomSensors.horizontal2->getDistanceTraveled();
    if (odomSensors.imu != nullptr) imuRaw = degToRad(odomSensors.imu->get_rotation());

    // calculate the change in sensor values
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;

    // update the previous sensor values
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheels
    // 2. Vertical tracking wheels
    // 3. Inertial Sensor
    // 4. Drivetrain
    float heading = odomPose.theta;
    // calculate the heading using the horizontal tracking wheels
    if (odomSensors.horizontal1 != nullptr && odomSensors.horizontal2 != nullptr)
        heading -= (deltaHorizontal1 - deltaHorizontal2) /
                   (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
    // else, if both vertical tracking wheels aren't substituted by the drivetrain, use the vertical tracking wheels
    else if (!odomSensors.vertical1->getType() && !odomSensors.vertical2->getType())
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    // else, if the inertial sensor exists, use it
    else if (odomSensors.imu != nullptr) heading += deltaImu;
    // else, use the the substituted tracking wheels
    else
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    lemlib::TrackingWheel* verticalWheel = nullptr;
    lemlib::TrackingWheel* horizontalWheel = nullptr;
    if (!odomSensors.vertical1->getType()) verticalWheel = odomSensors.vertical1;
    else if (!odomSensors.vertical2->getType()) verticalWheel = odomSensors.vertical2;
    else verticalWheel = odomSensors.vertical1;
    if (odomSensors.horizontal1 != nullptr) horizontalWheel = odomSensors.horizontal1;
    else if (odomSensors.horizontal2 != nullptr) horizontalWheel = odomSensors.horizontal2;
    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = verticalWheel->getDistanceTraveled();
    if (horizontalWheel != nullptr) rawHorizontal = horizontalWheel->getDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->getOffset();

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;
    
    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }
    
    // MCL START
    lemlib::Pose prevPose = odomPose;
    
    // Precompute trig once
    const float s = std::sin(avgHeading);
    const float c = std::cos(avgHeading);

    // Field-frame delta from local
    const float dxField = localY * s + localX * -c;
    const float dyField = localX * s + localY *  c;  
    
    const float deltaS = std::hypot(dxField, dyField);
    const float deltaT = std::fabs(deltaHeading);
    float sigmaXY;
    if (paused) {
        // Only add minor amounts of error to positions if we are paused
        sigmaXY = SIGMA0_XY;
    } else {
        sigmaXY = SIGMA0_XY + K_DIST_XY * deltaS + K_TURN_XY * deltaT;
    }
    
    // 1) Motion update: move every particle
    for (auto& p : particles) {
        p.adjustPose(dxField, dyField, sigmaXY);
        // printf("(%f, %f, %f)", odomPose.x + dxField - p.pose_.x, odomPose.y + dyField - p.pose_.y, p.weight_);
    }
    
    // TODO -> if robot rotates in place this never executes add a delta heading value
    // Possibly sensor update once every 0.5s even if we standing still to prevent particles from spreading out too much
    // const bool shouldDoSensor = (deltaS > 0.01f) || (pros::millis() - prev_time > 200);
    const bool shouldDoSensor = !paused;
    if (shouldDoSensor) {
        prev_time = pros::millis();
        // 2) Sensor update: update particle weights
        float zF = front_distance.get() / 25.4;
        float zL = left_distance.get() / 25.4;
        float zR = right_distance.get() / 25.4;

        int cF = front_distance.get_confidence();
        // int cL = left_distance.get_confidence();
        // int cR = right_distance.get_confidence();

        frontConf = cF;
    
        for (auto& p : particles) {
            p.sensorUpdate(zF, zL, zR, heading);
        }
        
        // 3) Normalize weights
        normalizeWeights(particles);
        
        // 4) Neff check -> resample if needed
        const double Neff = effectiveSampleSize(particles);
        const double N = static_cast<double>(particles.size());
        
        // Typical threshold: 0.5N (tune)
        // printf("N-Effective: %f, Threshold, %f\n", Neff, 0.5 * N);
        if (Neff < 0.5 * N) {
            particles = systematicResample(particles);
    
            // Optional: "roughening" / jitter here to prevent duplicates
            // for (auto& p : particles) p.addSmallJitter(...);
        }
        
    } else {
        // If you did not do sensor update, keep weights as-is (often uniform).
        // You could optionally skip resampling entirely here (recommended).
    }
    
    // 5) Get weighted mean of particle positions
    auto [meanX, meanY] = weightedMeanXY(particles);
    odomPose.x = meanX;
    odomPose.y = meanY;
    odomPose.theta = heading;
    
    // OLD calculate global x and y
    oldOdomPoseCalculation.x += dxField;
    oldOdomPoseCalculation.y += dyField;
    oldOdomPoseCalculation.theta = heading;

    // pros::lcd::print(0, "MCL X: %f Y: %f", odomPose.x, odomPose.y);
	// pros::lcd::print(1, "Theta: %f", odomPose.theta);
	// pros::lcd::print(2, "OLD X: %f Y: %f", oldOdomPoseCalculation.x, oldOdomPoseCalculation.y);


    // printf("MCL X:%f Y:%f ||| OLD X:%f Y:%f", odomPose.x, odomPose.y, oldOdomPoseCalculation.x, oldOdomPoseCalculation.y);
    
    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    // calculate local speed
    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);

    updateTime = pros::micros() - updateStartTime;
}

static std::pair<float, float> lemlib::weightedMeanXY(const std::vector<Particle>& particles) {
    double sumW = 0.0;
    double sumX = 0.0;
    double sumY = 0.0;

    for (const auto& p : particles) {
        const double w = p.weight_;
        sumW += w;
        sumX += w * p.pose_.x;
        sumY += w * p.pose_.y;
    }

    if (sumW <= 1e-12) {
        // Degenerate fallback: unweighted mean
        sumX = 0.0;
        sumY = 0.0;
        for (const auto& p : particles) {
            sumX += p.pose_.x;
            sumY += p.pose_.y;
        }
        const double invN = 1.0 / std::max<size_t>(1, particles.size());
        return {static_cast<float>(sumX * invN), static_cast<float>(sumY * invN)};
    }

    return {static_cast<float>(sumX / sumW), static_cast<float>(sumY / sumW)};
}


static std::vector<Particle> lemlib::systematicResample(const std::vector<Particle>& particles) {
    const int N = static_cast<int>(particles.size());
    std::vector<Particle> out;
    out.reserve(N);

    // Build CDF
    std::vector<float> cdf(N);
    float cum = 0.0f;
    for (int i = 0; i < N; i++) {
        cum += particles[i].weight_;
        cdf[i] = cum;
    }
    // Ensure last is exactly 1 (helps with floating point edge cases)
    cdf[N - 1] = 1.0f;

    // One random offset r in [0, 1/N)
    // TODO check if we can reuse the particle random device
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> unif(0.0f, 1.0f / N);
    const float r = unif(rng);

    int i = 0;
    for (int k = 0; k < N; k++) {
        const float u = r + (static_cast<float>(k) / N);

        while (i < N - 1 && u > cdf[i]) i++;

        out.push_back(particles[i]);     // copy particle i
        out.back().weight_ = 1.0f / N;   // reset weight after resampling
    }

    return out;
}

static double lemlib::effectiveSampleSize(const std::vector<Particle>& particles) {
    double sumSq = 0.0;
    for (const auto& p : particles) {
        const double w = p.weight_;
        sumSq += w * w;
    }
    if (sumSq <= 1e-18) return 0.0;
    return 1.0 / sumSq;
}

static void lemlib::normalizeWeights(std::vector<Particle>& particles) {
    double sumW = 0.0;
    for (auto& p : particles) sumW += p.weight_;

    if (sumW <= 1e-12) {
        // Degenerate case: reset to uniform
        const float w = 1.0f / static_cast<float>(particles.size());
        for (auto& p : particles) p.weight_ = w;
        return;
    }

    const double invSum = 1.0 / sumW;
    for (auto& p : particles) p.weight_ = static_cast<float>(p.weight_ * invSum);
}

void lemlib::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            prev_time = pros::millis();
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}

void lemlib::initParticles() {
    particles.clear();
    particles.reserve(mclSettings.particleCount);
    
    for (int i = 0; i < mclSettings.particleCount; ++i) {
        particles.emplace_back(odomPose, 1.0f / mclSettings.particleCount);
        particles[i].addError(MAX_START_POS_ERROR_IN / 3.0f);
    }
}
