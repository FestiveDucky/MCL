// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include <random>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
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

namespace {
constexpr float MM_TO_IN = 1.0f / 25.4f;

std::vector<std::unique_ptr<pros::Distance>> distanceSensors;
std::vector<Particle::SensorObservation> sensorObservations;

std::unique_ptr<pros::Distance> makeDistanceSensor(int port) {
    if (port <= 0) return nullptr;
    return std::make_unique<pros::Distance>(port);
}

void configureDistanceSensors() {
    distanceSensors.clear();
    distanceSensors.reserve(mclSettings.distanceSensors.size());
    for (const auto& sensorConfig : mclSettings.distanceSensors) {
        distanceSensors.emplace_back(makeDistanceSensor(sensorConfig.port));
    }

    sensorObservations.clear();
    sensorObservations.reserve(mclSettings.distanceSensors.size());
}

float readDistanceInches(const std::unique_ptr<pros::Distance>& sensor) {
    if (sensor == nullptr) return std::numeric_limits<float>::quiet_NaN();
    return sensor->get() * MM_TO_IN;
}

int readDistanceConfidence(const std::unique_ptr<pros::Distance>& sensor) {
    if (sensor == nullptr) return 0;
    return sensor->get_confidence();
}

bool inRange(float value, float min, float max) {
    return value >= min && value <= max;
}

float clampf(float value, float min, float max) {
    return std::max(min, std::min(value, max));
}

float confidenceToUnit(int confidence) {
    if (!mclSettings.useSensorConfidence) return 1.0f;
    const float denom = std::max(1e-6f, mclSettings.sensorConfMax);
    return clampf(static_cast<float>(confidence) / denom, 0.0f, 1.0f);
}
} // namespace

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

std::size_t lemlib::getDistanceSensorCount() {
    return distanceSensors.size();
}

float lemlib::getDistanceInchesByIndex(std::size_t index) {
    if (index >= distanceSensors.size()) return std::numeric_limits<float>::quiet_NaN();
    return readDistanceInches(distanceSensors[index]);
}

void lemlib::setMCLSettings(const lemlib::MCLSettings& settings) {
    mclSettings = settings;
    configureDistanceSensors();
}

const lemlib::MCLSettings& lemlib::getMCLSettings() {
    return mclSettings;
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
    const float deltaSForNoise = mclSettings.clampDeltaSForNoise ? std::min(deltaS, mclSettings.maxDeltaSForNoise) : deltaS;
    float sigmaXY;
    if (paused) {
        // Only add minor amounts of error to positions if we are paused
        sigmaXY = mclSettings.sigma0XY / 2;
    } else {
        sigmaXY = mclSettings.sigma0XY + mclSettings.kDistXY * deltaSForNoise + mclSettings.kTurnXY * deltaT;
        if (mclSettings.clampSigmaXY) sigmaXY = std::min(sigmaXY, mclSettings.maxSigmaXY);
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
    float estX = odomPose.x;
    float estY = odomPose.y;
    if (shouldDoSensor) {
        prev_time = pros::millis();
        // 2) Sensor update: build observations once per odom iteration.
        sensorObservations.clear();
        frontConf = 0;

        const float sh = std::sin(heading);
        const float ch = std::cos(heading);

        const std::size_t sensorCount = std::min(distanceSensors.size(), mclSettings.distanceSensors.size());
        for (std::size_t i = 0; i < sensorCount; i++) {
            const auto& sensor = distanceSensors[i];
            if (sensor == nullptr) continue;

            const auto& sensorConfig = mclSettings.distanceSensors[i];
            const float measuredDistanceIn = readDistanceInches(sensor);
            const int confidence = readDistanceConfidence(sensor);

            if (i == 0) frontConf = static_cast<std::uint32_t>(confidence);

            const bool hasHit = inRange(measuredDistanceIn, mclSettings.zMin, mclSettings.zMax);
            const bool hasNoHit = mclSettings.useNoHitModel && std::isfinite(measuredDistanceIn) &&
                                  measuredDistanceIn > mclSettings.zMax;
            if (!hasHit && !hasNoHit) continue;

            // Transform mount offsets and ray direction once per update. These values are reused for every particle.
            const float mountOffsetX = ch * sensorConfig.mount.xOffset + sh * sensorConfig.mount.yOffset;
            const float mountOffsetY = -sh * sensorConfig.mount.xOffset + ch * sensorConfig.mount.yOffset;
            const float rayAngle = heading + sensorConfig.mount.headingOffset;
            const float rayDirX = std::sin(rayAngle);
            const float rayDirY = std::cos(rayAngle);

            sensorObservations.push_back({measuredDistanceIn, confidenceToUnit(confidence), hasHit, hasNoHit,
                                          mountOffsetX, mountOffsetY, rayDirX, rayDirY});
        }

        if (!sensorObservations.empty()) {
            for (auto& p : particles) {
                p.sensorUpdate(sensorObservations);
            }

            // 3) Normalize weights
            normalizeWeights(particles);

            // Estimate pose from the posterior BEFORE resampling resets weights.
            auto [preResampleX, preResampleY] = weightedMeanXY(particles);
            estX = preResampleX;
            estY = preResampleY;

            // 4) Neff check -> resample if needed
            const double Neff = effectiveSampleSize(particles);
            const double N = static_cast<double>(particles.size());
            const double neffThreshold = static_cast<double>(mclSettings.neffResampleThreshold) * N;

            // Typical threshold: 0.5N (tune via mclSettings.neffResampleThreshold)
            // printf("N-Effective: %f, Threshold, %f\n", Neff, neffThreshold);
            if (Neff < neffThreshold) {
                particles = systematicResample(particles);

                // Optional: "roughening" / jitter here to prevent duplicates
                // for (auto& p : particles) p.addSmallJitter(...);
            }
        } else {
            auto [noSensorX, noSensorY] = weightedMeanXY(particles);
            estX = noSensorX;
            estY = noSensorY;
        }
    } else {
        // If you did not do sensor update, keep weights as-is (often uniform).
        // You could optionally skip resampling entirely here (recommended).
        auto [noSensorX, noSensorY] = weightedMeanXY(particles);
        estX = noSensorX;
        estY = noSensorY;
    }
    
    // 5) Apply filtered estimate
    odomPose.x = estX;
    odomPose.y = estY;
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
    if (particles.empty()) return {odomPose.x, odomPose.y};

    // Seed from the previous filtered estimate to stay on the same mode.
    const double seedX = odomPose.x;
    const double seedY = odomPose.y;
    double muX = seedX;
    double muY = seedY;

    // 1) Mean-shift with Epanechnikov kernel (no sqrt in inner loop).
    const double h = mclSettings.estMsBandwidth;
    const double h2 = h * h;
    const double epsStop2 = mclSettings.estMsEpsStop * mclSettings.estMsEpsStop;
    auto runMeanShift = [&](double startX, double startY, double& outX, double& outY) {
        outX = startX;
        outY = startY;
        for (int k = 0; k < mclSettings.estMsIters; k++) {
            double sumW = 0.0;
            double sumX = 0.0;
            double sumY = 0.0;

            for (const auto& p : particles) {
                const double dx = p.pose_.x - outX;
                const double dy = p.pose_.y - outY;
                const double r2 = dx * dx + dy * dy;

                const double t = 1.0 - (r2 / h2);
                if (t <= 0.0) continue;

                const double wk = p.weight_ * t;
                sumW += wk;
                sumX += wk * p.pose_.x;
                sumY += wk * p.pose_.y;
            }

            if (sumW <= 1e-12) return false;

            const double nextX = sumX / sumW;
            const double nextY = sumY / sumW;
            const double ddx = nextX - outX;
            const double ddy = nextY - outY;
            outX = nextX;
            outY = nextY;
            if ((ddx * ddx + ddy * ddy) <= epsStop2) break;
        }
        return true;
    };

    if (!runMeanShift(seedX, seedY, muX, muY)) {
        // If the previous estimate is outside the posterior support, re-seed from
        // the most likely particle instead of freezing at the stale pose.
        const auto bestIt = std::max_element(
            particles.begin(),
            particles.end(),
            [](const Particle& a, const Particle& b) { return a.weight_ < b.weight_; });
        if (bestIt == particles.end()) return {static_cast<float>(seedX), static_cast<float>(seedY)};

        muX = bestIt->pose_.x;
        muY = bestIt->pose_.y;
        if (!runMeanShift(muX, muY, muX, muY)) {
            return {static_cast<float>(muX), static_cast<float>(muY)};
        }
    }

    // 2) Optional Huber local refinement to reduce tail/outlier pull.
    double robustX = muX;
    double robustY = muY;
    if (mclSettings.estUseHuberRefinement) {
        const double rGate = mclSettings.estHuberGateMult * h;
        const double rGate2 = rGate * rGate;
        const double delta = mclSettings.estHuberDeltaMult * h;
        const double delta2 = delta * delta;

        for (int k = 0; k < mclSettings.estHuberIters; k++) {
            double sumW = 0.0;
            double sumX = 0.0;
            double sumY = 0.0;

            for (const auto& p : particles) {
                const double dx = p.pose_.x - robustX;
                const double dy = p.pose_.y - robustY;
                const double r2 = dx * dx + dy * dy;
                if (r2 > rGate2) continue;
                if (p.weight_ <= 0.0) continue;

                // Huber influence: mean-like near center, downweights large residuals.
                double hub = 1.0;
                if (r2 > delta2) {
                    const double r = std::sqrt(r2);
                    hub = (r > 1e-12) ? (delta / r) : 1.0;
                }

                const double ww = p.weight_ * hub;
                sumW += ww;
                sumX += ww * p.pose_.x;
                sumY += ww * p.pose_.y;
            }

            if (sumW <= 1e-12) break;

            const double nextX = sumX / sumW;
            const double nextY = sumY / sumW;
            const double ddx = nextX - robustX;
            const double ddy = nextY - robustY;
            robustX = nextX;
            robustY = nextY;
            if ((ddx * ddx + ddy * ddy) <= 1e-4) break;
        }
    }

    if (!mclSettings.estUseEmaSmoothing) {
        return {static_cast<float>(robustX), static_cast<float>(robustY)};
    }

    // 3) Adaptive EMA smoothing based on local spread around robust estimate.
    const double varGate = 2.0 * h;
    const double varGate2 = varGate * varGate;
    double sumWVar = 0.0;
    double sumR2 = 0.0;
    for (const auto& p : particles) {
        const double dx = p.pose_.x - robustX;
        const double dy = p.pose_.y - robustY;
        const double r2 = dx * dx + dy * dy;
        if (r2 > varGate2) continue;
        sumWVar += p.weight_;
        sumR2 += p.weight_ * r2;
    }

    // sigma is RMS local spread (inches); larger sigma => more smoothing.
    const double sigma = (sumWVar > 1e-12) ? std::sqrt(sumR2 / sumWVar) : mclSettings.estSigmaHi;
    const double sigmaSpan = std::max(1e-9f, mclSettings.estSigmaHi - mclSettings.estSigmaLo);
    const double t = std::max(0.0, std::min(1.0, (sigma - mclSettings.estSigmaLo) / sigmaSpan));
    double alpha = mclSettings.estAlphaMin + t * (mclSettings.estAlphaMax - mclSettings.estAlphaMin);

    // Innovation gate: suppress one-frame jumps.
    const double innovation = std::hypot(robustX - seedX, robustY - seedY);
    if (innovation > mclSettings.estJumpThresh) alpha = std::max(alpha, static_cast<double>(mclSettings.estAlphaJump));

    // printf("Innovation: %f, sigma: %f, alpha: %f\n", innovation, sigma, alpha);

    const double estX = alpha * seedX + (1.0 - alpha) * robustX;
    const double estY = alpha * seedY + (1.0 - alpha) * robustY;
    return {static_cast<float>(estX), static_cast<float>(estY)};
}


static std::vector<Particle> lemlib::systematicResample(const std::vector<Particle>& particles) {
    const int N = static_cast<int>(particles.size());
    std::vector<Particle> out;
    out.reserve(N);

    // Build CDF
    std::vector<double> cdf(N);
    double cum = 0.0;
    for (int i = 0; i < N; i++) {
        cum += particles[i].weight_;
        cdf[i] = cum;
    }
    // Ensure last is exactly 1 (helps with floating point edge cases)
    cdf[N - 1] = 1.0;

    // One random offset r in [0, 1/N)
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> unif(0.0, 1.0 / N);
    const double r = unif(rng);

    int i = 0;
    for (int k = 0; k < N; k++) {
        const double u = r + (static_cast<double>(k) / N);

        while (i < N - 1 && u > cdf[i]) i++;

        out.push_back(particles[i]);     // copy particle i
        out.back().weight_ = 1.0 / N;   // reset weight after resampling
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
    if (particles.empty()) return;

    double sumW = 0.0;
    for (auto& p : particles) sumW += p.weight_;

    if (sumW <= 1e-12) {
        // Degenerate case: reset to uniform
        const double w = 1.0 / static_cast<double>(particles.size());
        for (auto& p : particles) p.weight_ = w;
        return;
    }

    const double invSum = 1.0 / sumW;
    for (auto& p : particles) p.weight_ = p.weight_ * invSum;
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
    const int particleCount = std::max(0, mclSettings.particleCount);
    if (particleCount == 0) return;

    particles.reserve(static_cast<std::size_t>(particleCount));

    for (int i = 0; i < particleCount; ++i) {
        particles.emplace_back(odomPose, 1.0 / static_cast<double>(particleCount));
        particles[i].addError(mclSettings.maxStartPosErrorIn / 3.0f);
    }
}
