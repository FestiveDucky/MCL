#pragma once

#include "pros/distance.hpp"
#include "main.h"

class MCLDistSensor {
public:
    MCLDistSensor(int port, int xOffset, int yOffset);
    pros::Distance sensor;
    int xOffset;
    int yOffset;
};