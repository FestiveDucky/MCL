#include "particle.h"

#include <algorithm>
#include <cmath>
#include <random>

#include "lemlib/chassis/odom.hpp"

namespace {
constexpr float SQRT3 = 1.73205080757f;

static thread_local std::mt19937 rng{static_cast<uint32_t>(std::random_device{}())};

static inline float sampleUniformSymmetric(float a, float b) {
    static thread_local std::uniform_real_distribution<float> dist;
    dist.param(std::uniform_real_distribution<float>::param_type(a, b));
    return dist(rng);
}

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

static inline bool inRange(float v, float lo, float hi) {
    return (v >= lo && v <= hi);
}

// Raycast from (x0,y0) in direction (dx,dy) to rectangle boundary.
// Returns smallest valid t >= 0 (distance).
static float raycastToFieldWalls(float x0, float y0, float dx, float dy, float wallMin, float wallMax) {
    float bestT = INFINITY;

    // ---- Vertical walls x = wallMin and x = wallMax ----
    if (std::fabs(dx) > 1e-6f) {
        float t = (wallMin - x0) / dx;
        if (t >= 0.0f) {
            const float y = y0 + t * dy;
            if (inRange(y, wallMin, wallMax)) bestT = std::min(bestT, t);
        }

        t = (wallMax - x0) / dx;
        if (t >= 0.0f) {
            const float y = y0 + t * dy;
            if (inRange(y, wallMin, wallMax)) bestT = std::min(bestT, t);
        }
    }

    // ---- Horizontal walls y = wallMin and y = wallMax ----
    if (std::fabs(dy) > 1e-6f) {
        float t = (wallMin - y0) / dy;
        if (t >= 0.0f) {
            const float x = x0 + t * dx;
            if (inRange(x, wallMin, wallMax)) bestT = std::min(bestT, t);
        }

        t = (wallMax - y0) / dy;
        if (t >= 0.0f) {
            const float x = x0 + t * dx;
            if (inRange(x, wallMin, wallMax)) bestT = std::min(bestT, t);
        }
    }

    return bestT;
}

static inline double likelihoodTriangle(float e, const lemlib::MCLSettings& cfg) {
    const float b = SQRT3 * cfg.sigmaD;

    // Avoid divide by zero if sigmaD is accidentally 0
    if (b <= 1e-6f) return 1.0f;

    const double t = 1.0 - (std::fabs(e) / b);
    return std::max(cfg.pFloor, std::max(0.0, t));
}
} // namespace

Particle::Particle(lemlib::Pose p, double w) : pose_(p.x, p.y, p.theta), weight_(w) {}
Particle::Particle() : pose_(0.0, 0.0, 0.0), weight_(1.0) {}

void Particle::addError(float sigma) {
    const float r = SQRT3 * sigma;
    pose_.x += sampleUniformSymmetric(-r, r);
    pose_.y += sampleUniformSymmetric(-r, r);
}

void Particle::adjustPose(float x, float y, float sigmaXY) {
    const auto& cfg = lemlib::getMCLSettings();
    const float activeFieldMargin = cfg.useFieldMargin ? cfg.fieldMarginIn : 0.0f;
    const float xMin = -cfg.fieldHalf + activeFieldMargin;
    const float xMax = cfg.fieldHalf - activeFieldMargin;
    const float yMin = -cfg.fieldHalf + activeFieldMargin;
    const float yMax = cfg.fieldHalf - activeFieldMargin;

    pose_.x += x;
    pose_.y += y;
    addError(sigmaXY);

    if (pose_.x > xMax || pose_.x < xMin || pose_.y > yMax || pose_.y < yMin) {
        if (cfg.clampOobParticles) {
            pose_.x = clampf(pose_.x, xMin, xMax);
            pose_.y = clampf(pose_.y, yMin, yMax);
            if (cfg.penalizeOobParticles) weight_ = std::max(weight_ * cfg.oobWeightMult, 1e-12);
        } else {
            pose_.x = sampleUniformSymmetric(xMin, xMax);
            pose_.y = sampleUniformSymmetric(yMin, yMax);
        }
    }
}

void Particle::sensorUpdate(const std::vector<SensorObservation>& observations) {
    if (observations.empty()) return;

    const auto& cfg = lemlib::getMCLSettings();
    const float wallMin = -cfg.fieldHalf;
    const float wallMax = cfg.fieldHalf;
    const float zMax = cfg.zMax;
    const double pFloor = cfg.pFloor;

    const float x = pose_.x;
    const float y = pose_.y;
    double wMult = 1.0;

    for (const auto& observation : observations) {
        const float sx = x + observation.originOffsetX;
        const float sy = y + observation.originOffsetY;

        const float zHat = raycastToFieldWalls(sx, sy, observation.rayDirX, observation.rayDirY, wallMin, wallMax);
        if (!std::isfinite(zHat)) continue;

        double prob = 1.0;
        if (observation.hasHit) {
            const float e = observation.distanceIn - zHat;
            prob = likelihoodTriangle(e, cfg);
        } else if (observation.hasNoHit) {
            // "No hit": penalize particles that expected a wall in range.
            prob = (zHat <= zMax) ? cfg.noHitPenalty : 1.0;
        }

        // Confidence blending is pre-scaled per sensor in odom for reuse across all particles.
        prob = observation.confidence01 * prob + (1.0f - observation.confidence01);
        wMult *= std::max(prob, pFloor);
    }

    weight_ *= wMult;
    weight_ = std::max(weight_, 1e-12);
}
