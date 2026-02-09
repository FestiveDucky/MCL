#pragma once

#include "lemlib/pose.hpp"
#include <vector>

class Particle {
  public:
    struct SensorObservation {
        float distanceIn;
        float confidence01;
        bool hasHit;
        bool hasNoHit;
        float originOffsetX;
        float originOffsetY;
        float rayDirX;
        float rayDirY;
    };

    Particle(lemlib::Pose p, double w);
    Particle();
    void adjustPose(float x, float y, float sigmaXY);
    void sensorUpdate(const std::vector<SensorObservation>& observations);
    void addError(float sigma);
    lemlib::Pose pose_;
    double weight_ = 1.0;
};
