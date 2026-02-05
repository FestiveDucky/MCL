#include "particle.h"
#include <random>


// ---------- Field bounds (inches) ----------
constexpr float X_MIN = -70.25f;
constexpr float X_MAX = 70.25f;  // set to your coordinate system
constexpr float Y_MIN = -70.25f;
constexpr float Y_MAX = 70.25f;

// ---------- Distance sensor validity ----------
constexpr float Z_MIN = 2.0f;     // min reliable range (in)
constexpr float Z_MAX = 70.0f;    // max reliable range (in)

// ---------- Likelihood model tuning ----------
constexpr float SIGMA_D = 5.0f;   // distance measurement std dev (in)
constexpr float P_FLOOR = 1e-2f;

constexpr float W_HIT   = 0.90f; // Only for Gaussian
constexpr float W_RAND  = 1-W_HIT; // Only for Gaussian

// ---------- Sensor mounting: robot frame offsets (inches) ----------
// Convention: robot frame x = right, y = forward.
// These are measured from robot center to the sensor origin point.
constexpr float FRONT_X_OFF = -4.75f;
constexpr float FRONT_Y_OFF = 7.0f;

constexpr float LEFT_X_OFF  = -4.75f;
constexpr float LEFT_Y_OFF  = 1.3f;

constexpr float RIGHT_X_OFF = 5.0f;
constexpr float RIGHT_Y_OFF = 2.75f;

// Sensor directions relative to robot forward
constexpr float PHI_FRONT = 0.0f;
constexpr float PHI_LEFT  = -static_cast<float>(M_PI) / 2.0f;
constexpr float PHI_RIGHT = static_cast<float>(M_PI) / 2.0f;


static thread_local std::mt19937 rng{
    static_cast<uint32_t>(std::random_device{}())
};

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

Particle::Particle(lemlib::Pose p, float w): pose_(p.x, p.y, p.theta), weight_(w) {};
Particle::Particle(): pose_(0.0, 0.0, 0.0), weight_(1) {};


void Particle::addError(float sigma) {
    // Normal Dist
    // pose_.x += sampleGaussian(sigma);
    // pose_.y += sampleGaussian(sigma);
    
    const float r = 1.73205080757 * sigma;
    pose_.x += sampleUniformSymmetric(-r, r);
    pose_.y += sampleUniformSymmetric(-r, r);
}

void Particle::adjustPose(float x, float y, float sigmaXY) {
    pose_.x += x;
    pose_.y += y;
    
    addError(sigmaXY);
    
    if (pose_.x > 70.25 || pose_.x < -70.25 || pose_.y > 70.25 || pose_.y < -70.25) {
        pose_.x = sampleUniformSymmetric(-70.25, 70.25);
        pose_.y = sampleUniformSymmetric(-70.25, 70.25);
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

    //Theta = 0, Sensor Angle: -1.570796, X: 46.601833, Y: -61.172958, Reading: 9.015748, Prediction: 9.095260, Prob: 0.990819
    // dx = cos(-pi/2) = 0, dy = sin(-pi/2) = -1
    // dx should be -1
    
    // ---- Vertical walls x = X_MIN and x = X_MAX ----
    if (std::fabs(dx) > 1e-6f) {
        // x = X_MIN
        {
            float t = (X_MIN - x0) / dx;
            if (t >= 0.0f) {
                float y = y0 + t * dy;
                if (inRange(y, Y_MIN, Y_MAX)) bestT = std::min(bestT, t);
            }
        }
        // x = X_MAX
        {
            float t = (X_MAX - x0) / dx;
            if (t >= 0.0f) {
                float y = y0 + t * dy;
                if (inRange(y, Y_MIN, Y_MAX)) bestT = std::min(bestT, t);
            }
        }
    }

    // ---- Horizontal walls y = Y_MIN and y = Y_MAX ----
    if (std::fabs(dy) > 1e-6f) {
        // y = Y_MIN
        {
            float t = (Y_MIN - y0) / dy;
            if (t >= 0.0f) {
                float x = x0 + t * dx;
                if (inRange(x, X_MIN, X_MAX)) bestT = std::min(bestT, t);
            }
        }
        // y = Y_MAX
        {
            float t = (Y_MAX - y0) / dy;
            if (t >= 0.0f) {
                float x = x0 + t * dx;
                if (inRange(x, X_MIN, X_MAX)) bestT = std::min(bestT, t);
            }
        }
    }

    return bestT;
}

static inline float distanceLikelihood(float e) {
    const float inv2sig2 = 1.0f / (2.0f * SIGMA_D * SIGMA_D);
    const float pHit = std::exp(-(e * e) * inv2sig2);
    const float pRand = 1.0f / Z_MAX;

    float p = W_HIT * pHit + W_RAND * pRand;
    return std::max(p, P_FLOOR);
}

static inline float likelihoodTriangle(float e) {
    const float b = 1.73205080757 * SIGMA_D;

    // Avoid divide by zero if sigma_d is accidentally 0
    if (b <= 1e-6f) return 1.0f;

    const float t = 1.0f - (std::fabs(e) / b);   // 82% of values fall within [0, b] with sqrt(3) * b being the maximum value
    return std::max(P_FLOOR, std::max(0.0f, t));
}

static inline float likelihoodUniformWindow(float e) {
    const float b = 1.73205080757 * SIGMA_D; // window half-width

    // Inside window -> strong match
    if (std::fabs(e) <= b) return 1.0f;

    // Outside window -> weak match (but not zero)
    return P_FLOOR;
}


void Particle::sensorUpdate(float zF, float zL, float zR, float headingRad) {
    bool vF = inRange(zF, Z_MIN, Z_MAX);
    bool vL = inRange(zL, Z_MIN, Z_MAX);
    bool vR = inRange(zR, Z_MIN, Z_MAX);
    
    // If no valid sensors, we learn nothing → do nothing
    if (!vF && !vL && !vR) return;
    
    // 2) Pull particle position (x_i, y_i)
    const float x = pose_.x;
    const float y = pose_.y;
    
    // 3) Multiply likelihood contributions from each valid sensor
    float wMult = 1.0f;
    
    auto updateOne = [&](bool valid, float z, float xOff, float yOff, float phi) {
        if (!valid) return;
    
        // 3a) Compute the sensor origin in field coordinates
        float sx, sy;
        sensorOriginField(xOff, yOff, x, y, headingRad, sx, sy);
    
        // 3b) Compute the ray direction in field coordinates
        const float a = headingRad + phi;
        const float dx = std::sin(a);
        const float dy = std::cos(a);
    
        // 3c) Predict measurement = distance to nearest wall along that ray
        const float zHat = raycastToFieldWalls(sx, sy, dx, dy);
    
        // If particle is outside the field or ray fails, ignore this sensor for this particle
        if (!std::isfinite(zHat)) return;
    
        // 3d) Convert measurement error to likelihood
        const float e = z - zHat;
        const float prob = likelihoodTriangle(e);
        // printf("Angle: %f, Sensor Angle: %f, X: %f, Y: %f, Reading: %f, Prediction: %f, Prob: %f\n", headingRad, phi, sx, sy, z, zHat, prob);
        wMult *=  prob; // NOTE: Can be replaced with the other likelihoods.

    };
    
    // Apply each sensor
    updateOne(vF, zF, FRONT_X_OFF, FRONT_Y_OFF, PHI_FRONT);
    updateOne(vL, zL, LEFT_X_OFF,  LEFT_Y_OFF,  PHI_LEFT);
    updateOne(vR, zR, RIGHT_X_OFF, RIGHT_Y_OFF, PHI_RIGHT);
    
    // 4) Apply multiplier to the particle's weight
    weight_ *= wMult;
    
    // Keep weights in a sane numeric range
    weight_ = std::max(weight_, 1e-10f);
}