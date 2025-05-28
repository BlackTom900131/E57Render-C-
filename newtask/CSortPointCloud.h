#pragma once
#include <vector>
#include <limits>
#include <cmath>
#include <omp.h>
#include <stdexcept>
#include "CGlobal.h"
class CSortPointCloud
{
public:
	std::vector<Point> m_src_points;
	std::vector<Point> m_dst_points;
	
	void sortPoints(double offsetX, double offsetY, double offsetZ);
	void removeSmallFragment(int minPointsPerGroup, double offset);

private:
	double m_minX, m_maxX;
	double m_minY, m_maxY;
	double m_minZ, m_maxZ;
	
	//individuall offset.
	//double m_offsetX, m_offsetY, m_offsetZ;
	//global offset when used for offset x,y,z are same.
	//double m_offsetG;

	void getMinMax();
	void removeSamePoints();
};