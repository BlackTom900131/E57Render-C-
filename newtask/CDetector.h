#pragma once
#include <vector>
#include "CGlobal.h"
class CDetector
{
public:
    // Detects points near the floor level within heightThreshold (e.g. 0.05m)
    std::vector<Point> detectFloor(const std::vector<Point>& points, double heightThreshold = 0.01, size_t minClusterSize = 50, bool selectZ = false, double valueZ = 0);
    std::vector<std::vector<Point>> detectFloorAll(const std::vector<Point>& points, double heightThreshold = 0.01, size_t minClusterSize = 50);
    std::vector<std::vector<Point>> detectWallX(const std::vector<Point>& points, double heightThreshold = 0.01, size_t minClusterSize = 50);
    std::vector<std::vector<Point>> detectWallY(const std::vector<Point>& points, double heightThreshold = 0.01, size_t minClusterSize = 50);
};