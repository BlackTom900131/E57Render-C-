#pragma once

#include <vector>
#include "resource.h"
// Define point structure
struct Point {
    double x, y, z;
    int label; // segmentation label
};

// Point cloud data
extern std::vector<Point> points;
extern std::vector<Point> pointsc;