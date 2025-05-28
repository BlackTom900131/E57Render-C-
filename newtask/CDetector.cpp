#include "CDetector.h"
#include <algorithm>
std::vector<Point> CDetector::detectFloor(const std::vector<Point>& points, double heightThreshold, size_t minClusterSize, bool selectZ, double valueZ) {
    double minZ;
    if (points.empty()) return {};
    if (selectZ) 
        minZ = valueZ;
    else{
        // 1. Find minimum Z (lowest point)
        minZ = points[0].z;
        for (const auto& pt : points) if (pt.z < minZ) minZ = pt.z;
    }
    // 2. Filter points near minZ
    std::vector<Point> floorCandidates;
    for (const auto& pt : points) {
        if (pt.z <= minZ + heightThreshold) {
            floorCandidates.push_back(pt);
        }
    }

    // 3. Check cluster size
    if (floorCandidates.size() < minClusterSize) {
        return {}; // Not enough points to consider a floor
    }

    return floorCandidates;
}

std::vector<std::vector<Point>> CDetector::detectFloorAll(const std::vector<Point>& points, double heightThreshold, size_t minClusterSize)
{
    if (points.empty()) return {};

    // 1. Sort points by Z
    std::vector<Point> sortedPoints = points;
    std::sort(sortedPoints.begin(), sortedPoints.end(), [](const Point& a, const Point& b) {
        return a.z < b.z;
        });

    std::vector<std::vector<Point>> floors;
    std::vector<Point> currentFloor;
    currentFloor.reserve(points.size()); // prevent reallocation

    double currentFloorZ = sortedPoints[0].z;
    int label = 1;

    for (auto& pt : sortedPoints) {
        if (pt.label >= 0) continue; // skip already labeled points

        if (pt.z <= currentFloorZ + heightThreshold) {
            pt.label = label;
            currentFloor.push_back(pt);
        }
        else {
            // finish the current floor
            if (currentFloor.size() >= minClusterSize) {
                floors.push_back(currentFloor);
                label++;
            }
            currentFloor.clear();
            currentFloorZ = pt.z;
            pt.label = label;
            currentFloor.push_back(pt);
        }
    }

    // handle the last group
    if (currentFloor.size() >= minClusterSize) {
        floors.push_back(currentFloor);
    }

    return floors;
}

std::vector<std::vector<Point>> CDetector::detectWallX(const std::vector<Point>& points, double heightThreshold, size_t minClusterSize)
{
    if (points.empty()) return {};

    // 1. Sort points by X
    std::vector<Point> sortedPoints = points;
    std::sort(sortedPoints.begin(), sortedPoints.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
        });

    std::vector<std::vector<Point>> walls;
    std::vector<Point> currentWall;
    currentWall.reserve(points.size()); // prevent reallocation

    double currentWallX = sortedPoints[0].x;
    int label = 1;

    for (auto& pt : sortedPoints) {
        if (pt.label >= 0) continue; // skip already labeled points

        if (pt.z <= currentWallX + heightThreshold) {
            pt.label = label;
            currentWall.push_back(pt);
        }
        else {
            // finish the current floor
            if (currentWall.size() >= minClusterSize) {
                walls.push_back(currentWall);
                label++;
            }
            currentWall.clear();
            currentWallX = pt.x;
            pt.label = label;
            currentWall.push_back(pt);
        }
    }

    // handle the last group
    if (currentWall.size() >= minClusterSize) {
        walls.push_back(currentWall);
    }

    return walls;
}

std::vector<std::vector<Point>> CDetector::detectWallY(const std::vector<Point>& points, double heightThreshold, size_t minClusterSize)
{
    if (points.empty()) return {};

    // 1. Sort points by Y
    std::vector<Point> sortedPoints = points;
    std::sort(sortedPoints.begin(), sortedPoints.end(), [](const Point& a, const Point& b) {
        return a.y < b.y;
        });

    std::vector<std::vector<Point>> walls;
    std::vector<Point> currentWall;
    currentWall.reserve(points.size()); // prevent reallocation

    double currentWallY = sortedPoints[0].y;
    int label = 1;

    for (auto& pt : sortedPoints) {
        if (pt.label >= 0) continue; // skip already labeled points

        if (pt.y <= currentWallY + heightThreshold) {
            pt.label = label;
            currentWall.push_back(pt);
        }
        else {
            // finish the current floor
            if (currentWall.size() >= minClusterSize) {
                walls.push_back(currentWall);
                label++;
            }
            currentWall.clear();
            currentWallY = pt.y;
            pt.label = label;
            currentWall.push_back(pt);
        }
    }

    // handle the last group
    if (currentWall.size() >= minClusterSize) {
        walls.push_back(currentWall);
    }

    return walls;
}
