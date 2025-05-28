#include "CSortPointCloud.h"
#include <unordered_set>
#include <unordered_map>

void CSortPointCloud::sortPoints(double offsetX, double offsetY, double offsetZ)
{
    // Offset validation
    if (offsetX <= 0 || offsetY <= 0 || offsetZ <= 0) {
        // Optionally log a warning or throw an exception
        return;  // Return empty result
    }
    // Calculate min, max value for every x,y,z -axis
    this->getMinMax();

    //Sort points by offset.
    if (!m_dst_points.empty()) {
        m_dst_points.clear();
    }
    for (const auto& pt : m_src_points) {
        Point snapped;
        snapped.x = std::floor((pt.x - m_minX) / offsetX) * offsetX + m_minX;
        snapped.y = std::floor((pt.y - m_minY) / offsetY) * offsetY + m_minY;
        snapped.z = std::floor((pt.z - m_minZ) / offsetZ) * offsetZ + m_minZ;
        snapped.label = -1;
        m_dst_points.push_back(snapped);
    }

    //Remove points
    removeSamePoints();

    return;
}

void CSortPointCloud::getMinMax()
{
    // Initialize with extreme values
    m_minX = m_minY = m_minZ = std::numeric_limits<double>::max();
    m_maxX = m_maxY = m_maxZ = std::numeric_limits<double>::lowest();

    // Declare thread-local minima/maxima
    double local_minX, local_maxX;
    double local_minY, local_maxY;
    double local_minZ, local_maxZ;

#pragma omp parallel private(local_minX, local_maxX, local_minY, local_maxY, local_minZ, local_maxZ)
    {
        // Initialize local minima/maxima for each thread
        local_minX = std::numeric_limits<double>::max();
        local_maxX = std::numeric_limits<double>::lowest();
        local_minY = std::numeric_limits<double>::max();
        local_maxY = std::numeric_limits<double>::lowest();
        local_minZ = std::numeric_limits<double>::max();
        local_maxZ = std::numeric_limits<double>::lowest();

#pragma omp for nowait
        for (long long i = 0; i < m_src_points.size(); ++i) {
            const auto& pt = m_src_points[i];

            if (pt.x < local_minX) local_minX = pt.x;
            if (pt.x > local_maxX) local_maxX = pt.x;

            if (pt.y < local_minY) local_minY = pt.y;
            if (pt.y > local_maxY) local_maxY = pt.y;

            if (pt.z < local_minZ) local_minZ = pt.z;
            if (pt.z > local_maxZ) local_maxZ = pt.z;
        }

        // Merge local results into global minima/maxima
#pragma omp critical
        {
            if (local_minX < m_minX) m_minX = local_minX;
            if (local_maxX > m_maxX) m_maxX = local_maxX;

            if (local_minY < m_minY) m_minY = local_minY;
            if (local_maxY > m_maxY) m_maxY = local_maxY;

            if (local_minZ < m_minZ) m_minZ = local_minZ;
            if (local_maxZ > m_maxZ) m_maxZ = local_maxZ;
        }
    }
}

void CSortPointCloud::removeSamePoints()
{
    std::unordered_set<Point, PointHash> unique_points;
    std::vector<Point> filtered;

    for (const auto& pt : m_dst_points) {
        if (unique_points.insert(pt).second) {
            filtered.push_back(pt);
        }
    }

    m_dst_points = std::move(filtered);
}

void CSortPointCloud::removeSmallFragment(int minPointsPerGroup, double offset) {
    if (offset <= 0 || minPointsPerGroup <= 0) return;

    std::unordered_map<GridKey, std::vector<Point>, GridHash> groups;

    // Step 1: Group points by grid key
    for (const auto& pt : m_dst_points) {
        GridKey key{
            static_cast<int>(pt.x / offset),
            static_cast<int>(pt.y / offset),
            static_cast<int>(pt.z / offset)
        };
        groups[key].push_back(pt);
    }

    // Step 2: Collect points only from valid groups
    std::vector<Point> filtered;
    for (const auto& [key, pts] : groups) {
        if (static_cast<int>(pts.size()) >= minPointsPerGroup) {
            filtered.insert(filtered.end(), pts.begin(), pts.end());
        }
    }

    // Step 3: Replace the original points
    m_dst_points = std::move(filtered);
}

