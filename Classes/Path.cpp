#include "Path.h"
#include "utils.h"
#include <stdlib.h>

std::list<cocos2d::Point> Path::CreateRandomPath(int   NumWaypoints,
										   double MinX,
										   double MinY,
										   double MaxX,
										   double MaxY)
{
	m_WayPoints.clear();

	double midX = (MaxX+MinX)/2.0;
	double midY = (MaxY+MinY)/2.0;

	double smaller = MIN(midX, midY);

	double spacing = TwoPi/(double)NumWaypoints;

	for (int i=0; i<NumWaypoints; ++i)
	{
		double RadialDist = RandInRange(smaller*0.2f, smaller);

		cocos2d::Point temp(RadialDist, 0.0f);

		Vec2DRotateAroundOrigin(temp, i*spacing);

		temp.x += midX; temp.y += midY;

		m_WayPoints.push_back(temp);

	}

	curWaypoint = m_WayPoints.begin();

	return m_WayPoints;
}
