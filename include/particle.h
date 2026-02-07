#pragma once

#include "lemlib/pose.hpp"

class Particle {
  public:
    Particle(lemlib::Pose p, float w);
    Particle();
    void adjustPose(float x, float y, float sigmaXY);
    void sensorUpdate(float zF, float zL, float zR, float headingRad, int cF = 63, int cL = 63, int cR = 63);
    void addError(float sigma);
    lemlib::Pose pose_;
    float weight_ = 1;
};
