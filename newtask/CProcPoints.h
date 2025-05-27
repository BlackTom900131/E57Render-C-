#pragma once
#include <vector>
#include <array>
#include "newtask.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointRGB;

class CProcPoints
{
public:
	std::vector<std::array<double, 3>> m_raw_points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	pcl::PointCloud<PointRGB>::Ptr m_colored_cloud;
	pcl::PointCloud<PointT>::Ptr m_cloud_remaining;

	CProcPoints();
	~CProcPoints();
	void convertPoints();
	void convertPointCloud();
	void segmentPlanes();
};
