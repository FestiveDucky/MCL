#include "mclDistSensor.h"


MCLDistSensor::MCLDistSensor(int port, int xOffset, int yOffset) : sensor(pros::Distance(port)), xOffset(xOffset), yOffset(yOffset) {}
