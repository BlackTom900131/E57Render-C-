#include "CProcPoints.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
CProcPoints::CProcPoints() : m_cloud(new pcl::PointCloud<pcl::PointXYZ>())
, m_colored_cloud(new pcl::PointCloud<PointRGB>())
, m_cloud_remaining(new pcl::PointCloud<PointT>())
{
    //m_cloud_remaining(new pcl::PointCloud<PointT>())
}

CProcPoints::~CProcPoints() {
}

void CProcPoints::convertPoints()
{
	for (const auto& pt : points) {
		m_raw_points.push_back({ pt.x, pt.y, pt.z });
	}
}
void CProcPoints::convertPointCloud()
{
    for (const auto& p : m_raw_points) {
        pcl::PointXYZ point;
        point.x = p[0];
        point.y = p[1];
        point.z = p[2];
        m_cloud->points.push_back(point);
    }
    m_cloud->width = static_cast<uint32_t>(m_cloud->points.size());
    m_cloud->height = 1; // unorganized point cloud
    m_cloud->is_dense = true;
}

void CProcPoints::segmentPlanes()
{
    int plane_id = 0;
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> colors = {
        {255, 0, 0},   // Red
        {0, 255, 0},   // Green
        {0, 0, 255},   // Blue
        {255, 255, 0}, // Yellow
        {0, 255, 255}, // Cyan
        {255, 0, 255}  // Magenta
    };
    *m_cloud_remaining = *m_cloud;

    while (m_cloud_remaining->points.size() > 100) {
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::ExtractIndices<PointT> extract;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(m_cloud_remaining);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
            break;

        // Extract the plane points
        pcl::PointCloud<PointT>::Ptr plane_points(new pcl::PointCloud<PointT>);
        extract.setInputCloud(m_cloud_remaining);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_points);

        // Color the plane points and add to colored_cloud
        auto [r, g, b] = colors[plane_id % colors.size()];
        for (const auto& pt : plane_points->points) {
            PointRGB cpt;
            cpt.x = pt.x;
            cpt.y = pt.y;
            cpt.z = pt.z;
            cpt.r = r;
            cpt.g = g;
            cpt.b = b;
            Point cp;
            cp.x = pt.x;
            cp.y = pt.y;
            cp.z = pt.z;
            cp.label = plane_id + 1;

            m_colored_cloud->points.push_back(cpt);
            pointsc.push_back(cp);
        }

        // Remove plane points from remaining cloud
        extract.setNegative(true);
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        extract.filter(*cloud_filtered);
        m_cloud_remaining = cloud_filtered;

        plane_id++;
        if (plane_id == 6)
            break;
    }

    // Add remaining (non-plane) points without color (white or gray)
    for (const auto& pt : m_cloud_remaining->points) {
        PointRGB cpt;
        cpt.x = pt.x;
        cpt.y = pt.y;
        cpt.z = pt.z;
        cpt.r = 200;
        cpt.g = 200;
        cpt.b = 200;
        Point cp;
        cp.x = pt.x;
        cp.y = pt.y;
        cp.z = pt.z;
        cp.label =  0;
        m_colored_cloud->points.push_back(cpt);
        pointsc.push_back(cp);
    }
}
