#ifndef PATH_H
#define PATH_H
//------------------------------------------------------------------------
//
//  Name:   Path.h
//
//  Desc:   class to define, manage, and traverse a path (defined by a series of 2D vectors)
//          
//
//  Author: Mat Buckland 2003 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------
#include <list>
#include <cassert>

#include "cocos2d.h"
#include "C2DMatrix.h"

class Path
{
private:

	std::list<cocos2d::Point>            m_WayPoints;

	//points to the current waypoint
	std::list<cocos2d::Point>::iterator  curWaypoint;

	//flag to indicate if the path should be looped
	//(The last waypoint connected to the first)
	bool                           m_bLooped;

public:

	Path():m_bLooped(false), curWaypoint(m_WayPoints.end()){}

	//constructor for creating a path with initial random waypoints. MinX/Y
	//& MaxX/Y define the bounding box of the path.
	Path(int    NumWaypoints,
		double MinX,
		double MinY,
		double MaxX,
		double MaxY,
		bool   looped):m_bLooped(looped)
	{
		CreateRandomPath(NumWaypoints, MinX, MinY, MaxX, MaxY);

		curWaypoint = m_WayPoints.begin();
	}

	void Vec2DRotateAroundOrigin(Point& v, double ang);
	//returns the current waypoint
	cocos2d::Point    CurrentWaypoint()const{assert(curWaypoint != m_WayPoints.end()); return *curWaypoint;}

	//returns true if the end of the list has been reached
	bool        Finished(){return !(curWaypoint != m_WayPoints.end());}

	//moves the iterator on to the next waypoint in the list
	inline void SetNextWaypoint();

	//creates a random path which is bound by rectangle described by
	//the min/max values
	std::list<cocos2d::Point> CreateRandomPath(int    NumWaypoints,
		double MinX,
		double MinY,
		double MaxX,
		double MaxY);


	void LoopOn(){m_bLooped = true;}
	void LoopOff(){m_bLooped = false;}

	//adds a waypoint to the end of the path
	void AddWayPoint(cocos2d::Point new_point);

	//methods for setting the path with either another Path or a list of vectors
	void Set(std::list<cocos2d::Point> new_path){m_WayPoints = new_path;curWaypoint = m_WayPoints.begin();}
	void Set(const Path& path){m_WayPoints=path.GetPath(); curWaypoint = m_WayPoints.begin();}


	void Clear(){m_WayPoints.clear();}

	std::list<cocos2d::Point> GetPath()const{return m_WayPoints;}

};




//-------------------- Methods -------------------------------------------

inline void Path::SetNextWaypoint()
{
	assert (m_WayPoints.size() > 0);

	if (++curWaypoint == m_WayPoints.end())
	{
		if (m_bLooped)
		{
			curWaypoint = m_WayPoints.begin(); 
		}
	}
}  

//-------------------------- Vec2DRotateAroundOrigin --------------------------
//
//  rotates a vector ang rads around the origin
//-----------------------------------------------------------------------------
inline void Path::Vec2DRotateAroundOrigin(Point& v, double ang)
{
	//create a transformation matrix
	C2DMatrix mat;

	//rotate
	mat.Rotate(ang);

	//now transform the object's vertices
	mat.TransformVector2Ds(v);
}



#endif