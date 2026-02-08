#include "particle.h"
#include <algorithm>
#include <cmath>
#include <random>

// ---------- Feature toggles ----------
constexpr bool MCL_CLAMP_OOB_PARTICLES = true;     // #1
constexpr bool MCL_PENALIZE_OOB_PARTICLES = true;  // #1
constexpr double OOB_WEIGHT_MULT = 1e-3;

constexpr bool MCL_USE_FIELD_MARGIN = true;       // #3
constexpr float FIELD_MARGIN_IN = 5.5f;            // Half of track width

constexpr bool MCL_USE_SENSOR_CONFIDENCE = true;   // #4
constexpr float SENSOR_CONF_MAX = 63.0f;

constexpr bool MCL_USE_NO_HIT_MODEL = true;        // #5
constexpr float NO_HIT_PENALTY = 0.05f;

// ---------- Field bounds (inches) ----------
constexpr float FIELD_HALF = 70.75f;
constexpr float WALL_X_MIN = -FIELD_HALF;
constexpr float WALL_X_MAX = FIELD_HALF;
constexpr float WALL_Y_MIN = -FIELD_HALF;
constexpr float WALL_Y_MAX = FIELD_HALF;
constexpr float ACTIVE_FIELD_MARGIN = MCL_USE_FIELD_MARGIN ? FIELD_MARGIN_IN : 0.0f;
constexpr float X_MIN = -FIELD_HALF + ACTIVE_FIELD_MARGIN;
constexpr float X_MAX = FIELD_HALF - ACTIVE_FIELD_MARGIN;
constexpr float Y_MIN = -FIELD_HALF + ACTIVE_FIELD_MARGIN;
constexpr float Y_MAX = FIELD_HALF - ACTIVE_FIELD_MARGIN;

// ---------- Distance sensor validity ----------
constexpr float Z_MIN = 0.1f;     // min reliable range (in)
constexpr float Z_MAX = 85.0f;    // max reliable range (in)

// ---------- Likelihood model tuning ----------
constexpr float SIGMA_D = 10.0f;   // distance measurement std dev (in)
constexpr double P_FLOOR = 1e-4;

constexpr float W_HIT = 0.90f; // Only for Gaussian
constexpr float W_RAND = 1 - W_HIT; // Only for Gaussian

// ---------- Sensor mounting: robot frame offsets (inches) ----------
// Convention: robot frame x = right, y = forward.
// These are measured from robot center to the sensor origin point.
constexpr float FRONT_X_OFF = -4.75f;
constexpr float FRONT_Y_OFF = 7.0f;

constexpr float LEFT_X_OFF = -4.75f;
constexpr float LEFT_Y_OFF = 1.3f;

constexpr float RIGHT_X_OFF = 5.0f;
constexpr float RIGHT_Y_OFF = 2.75f;

// Sensor directions relative to robot forward
constexpr float PHI_FRONT = 0.0f;
constexpr float PHI_LEFT = -static_cast<float>(M_PI) / 2.0f;
constexpr float PHI_RIGHT = static_cast<float>(M_PI) / 2.0f;

static thread_local std::mt19937 rng{static_cast<uint32_t>(std::random_device{}())};

static inline float sampleGaussian(float sigma) {
    static thread_local std::normal_distribution<float> dist;
    dist.param(std::normal_distribution<float>::param_type(0.0f, sigma));
    return dist(rng);
}

static inline float sampleUniformSymmetric(float a, float b) {
    static thread_local std::uniform_real_distribution<float> dist;
    dist.param(std::uniform_real_distribution<float>::param_type(a, b));
    return dist(rng);
}

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

static inline float confidenceToUnit(int conf) {
    if (!MCL_USE_SENSOR_CONFIDENCE) return 1.0f;
    return clampf(static_cast<float>(conf) / SENSOR_CONF_MAX, 0.0f, 1.0f);
}

Particle::Particle(lemlib::Pose p, double w) : pose_(p.x, p.y, p.theta), weight_(w) {}
Particle::Particle() : pose_(0.0, 0.0, 0.0), weight_(1.0) {}

void Particle::addError(float sigma) {
    // Normal Dist
    // pose_.x += sampleGaussian(sigma);
    // pose_.y += sampleGaussian(sigma);

    const float r = 1.73205080757f * sigma;
    pose_.x += sampleUniformSymmetric(-r, r);
    pose_.y += sampleUniformSymmetric(-r, r);
}

void Particle::adjustPose(float x, float y, float sigmaXY) {
    pose_.x += x;
    pose_.y += y;

    addError(sigmaXY);

    if (pose_.x > X_MAX || pose_.x < X_MIN || pose_.y > Y_MAX || pose_.y < Y_MIN) {
        if (MCL_CLAMP_OOB_PARTICLES) {
            pose_.x = clampf(pose_.x, X_MIN, X_MAX);
            pose_.y = clampf(pose_.y, Y_MIN, Y_MAX);
            if (MCL_PENALIZE_OOB_PARTICLES) {
                weight_ = std::max(weight_ * OOB_WEIGHT_MULT, 1e-12);
            }
        } else {
            // My old code
            pose_.x = sampleUniformSymmetric(X_MIN, X_MAX);
            pose_.y = sampleUniformSymmetric(Y_MIN, Y_MAX);
        }
    }
}

static inline bool inRange(float v, float lo, float hi) {
    return (v >= lo && v <= hi);
}

static inline void sensorOriginField(float xr, float yr,
                                     float x, float y, float headingRad,
                                     float& sx, float& sy) {
    const float sh = std::sin(headingRad);
    const float ch = std::cos(headingRad);

    // forward (field) = (sin h, cos h)
    // right   (field) = (cos h, -sin h)
    sx = x + ch * xr + sh * yr;
    sy = y - sh * xr + ch * yr;
}

// Raycast from (x0,y0) in direction (dx,dy) to rectangle boundary.
// Returns smallest valid t >= 0 (distance).
static float raycastToFieldWalls(float x0, float y0, float dx, float dy) {
    float bestT = INFINITY;

    // Raycast uses true wall coordinates (not margin-shrunk particle bounds).
    // Predicted sensor distances should correspond to physical field walls.
    // ---- Vertical walls x = WALL_X_MIN and x = WALL_X_MAX ----
    if (std::fabs(dx) > 1e-6f) {
        // x = WALL_X_MIN
        {
            float t = (WALL_X_MIN - x0) / dx;
            if (t >= 0.0f) {
                float y = y0 + t * dy;
                if (inRange(y, WALL_Y_MIN, WALL_Y_MAX)) bestT = std::min(bestT, t);
            }
        }
        // x = WALL_X_MAX
        {
            float t = (WALL_X_MAX - x0) / dx;
            if (t >= 0.0f) {
                float y = y0 + t * dy;
                if (inRange(y, WALL_Y_MIN, WALL_Y_MAX)) bestT = std::min(bestT, t);
            }
        }
    }

    // ---- Horizontal walls y = WALL_Y_MIN and y = WALL_Y_MAX ----
    if (std::fabs(dy) > 1e-6f) {
        // y = WALL_Y_MIN
        {
            float t = (WALL_Y_MIN - y0) / dy;
            if (t >= 0.0f) {
                float x = x0 + t * dx;
                if (inRange(x, WALL_X_MIN, WALL_X_MAX)) bestT = std::min(bestT, t);
            }
        }
        // y = WALL_Y_MAX
        {
            float t = (WALL_Y_MAX - y0) / dy;
            if (t >= 0.0f) {
                float x = x0 + t * dx;
                if (inRange(x, WALL_X_MIN, WALL_X_MAX)) bestT = std::min(bestT, t);
            }
        }
    }

    return bestT;
}

static inline double distanceLikelihood(float e) {
    const float inv2sig2 = 1.0f / (2.0f * SIGMA_D * SIGMA_D);
    const float pHit = std::exp(-(e * e) * inv2sig2);
    const float pRand = 1.0f / Z_MAX;

    const double p = static_cast<double>(W_HIT * pHit + W_RAND * pRand);
    return std::max(p, P_FLOOR);
}

static inline double likelihoodTriangle(float e) {
    const float b = 1.73205080757f * SIGMA_D;

    // Avoid divide by zero if sigma_d is accidentally 0
    if (b <= 1e-6f) return 1.0f;

    const double t = 1.0 - (std::fabs(e) / b);
    return std::max(P_FLOOR, std::max(0.0, t));
}

static inline double likelihoodUniformWindow(float e) {
    const float b = 1.73205080757f * SIGMA_D;

    // Inside window -> strong match
    if (std::fabs(e) <= b) return 1.0f;

    // Outside window -> weak match (but not zero)
    return P_FLOOR;
}

void Particle::sensorUpdate(float zF, float zL, float zR, float headingRad, int cF, int cL, int cR) {
    const bool vF = inRange(zF, Z_MIN, Z_MAX);
    const bool vL = inRange(zL, Z_MIN, Z_MAX);
    const bool vR = inRange(zR, Z_MIN, Z_MAX);

    const bool nF = MCL_USE_NO_HIT_MODEL && std::isfinite(zF) && (zF > Z_MAX);
    const bool nL = MCL_USE_NO_HIT_MODEL && std::isfinite(zL) && (zL > Z_MAX);
    const bool nR = MCL_USE_NO_HIT_MODEL && std::isfinite(zR) && (zR > Z_MAX);

    // If no usable sensors, we learn nothing.
    if (!vF && !vL && !vR && !nF && !nL && !nR) return;

    const float x = pose_.x;
    const float y = pose_.y;

    double wMult = 1.0;

    auto updateOne = [&](bool valid, bool noHit, float z, float xOff, float yOff, float phi, int conf) {
        if (!valid && !noHit) return;

        // Sensor origin in field coordinates.
        float sx, sy;
        sensorOriginField(xOff, yOff, x, y, headingRad, sx, sy);

        // Sensor ray direction in field coordinates.
        const float a = headingRad + phi;
        const float dx = std::sin(a);
        const float dy = std::cos(a);

        // Predicted distance to nearest wall.
        const float zHat = raycastToFieldWalls(sx, sy, dx, dy);
        if (!std::isfinite(zHat)) return;

        double prob = 1.0;
        if (valid) {
            const float e = z - zHat;
            prob = likelihoodTriangle(e);
        } else {
            // "No hit": penalize particles that expected a wall in range.
            prob = (zHat <= Z_MAX) ? NO_HIT_PENALTY : 1.0;
        }

        if (MCL_USE_SENSOR_CONFIDENCE) {
            // Blend toward neutral likelihood when confidence is low.
            const float conf01 = confidenceToUnit(conf);
            prob = conf01 * prob + (1.0f - conf01);
        }

        wMult *= std::max(prob, P_FLOOR);
    };

    updateOne(vF, nF, zF, FRONT_X_OFF, FRONT_Y_OFF, PHI_FRONT, cF);
    updateOne(vL, nL, zL, LEFT_X_OFF, LEFT_Y_OFF, PHI_LEFT, cL);
    updateOne(vR, nR, zR, RIGHT_X_OFF, RIGHT_Y_OFF, PHI_RIGHT, cR);

    weight_ *= wMult;
    weight_ = std::max(weight_, 1e-12);
}
