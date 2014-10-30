#pragma once
#include "cocos2d.h"
#include "Vehicle.h"
#include "C2DMatrix.h"
#include "Path.h"

USING_NS_CC;
class Vehicle;
struct Obstacle;
struct Wall2D;

//--------------------------- Constants ----------------------------------

//the radius of the constraining circle for the wander behavior
const double WanderRad    = 1.2;
//distance the wander circle is projected in front of the agent
const double WanderDist   = 1.5;
//the maximum amount of displacement along the circle each frame
const double WanderJitterPerSec = 360.0;

class SteeringBehavior : public Ref
{
private:
	//a pointer to the owner of this instance
	Vehicle*     m_pVehicle;   

	//the steering force created by the combined effect of all
	//the selected behaviors
	Point    m_vSteeringForce;
	//the current position on the wander circle the agent is
	//attempting to steer towards
	Point     m_vWanderTarget; 

	//explained above
	double        m_dWanderJitter;
	double        m_dWanderRadius;
	double        m_dWanderDistance;

	//length of the 'detection box' utilized in obstacle avoidance
	double                 m_dDBoxLength;


	//a vertex buffer to contain the feelers rqd for wall avoidance  
	std::vector<Point> m_Feelers;

	//the length of the 'feeler/s' used in wall detection
	double                 m_dWallDetectionFeelerLength;

	 bool      AccumulateForce(Point &sf, Point ForceToAdd);

	 bool m_seekOn;
	 bool m_hideOn;
	 bool m_obstacleAvoidanceOn;
	 bool m_wanderOn;
	 bool m_evadeOn;
	 bool m_pursuitOn;
	 bool m_arriveOn;
	 bool m_fleeOn;
	 bool m_interposeOn;
	 bool m_wallAvoidanceOn;
	 bool m_followPathOn;

	 	//use these values to tweak the amount that each steering force
	 	//contributes to the total steering force
	 double	SeparationWeight;          
	 double	AlignmentWeight;           
	 double	CohesionWeight;            
	 double	ObstacleAvoidanceWeight;   
	 double	WallAvoidanceWeight;       
	 double	WanderWeight;              
	 double	SeekWeight;                
	 double	FleeWeight;               
	 double	ArriveWeight;             
	 double	PursuitWeight;            
	 double	OffsetPursuitWeight;      
	 double	InterposeWeight;          
	 double	HideWeight;               
	 double	EvadeWeight;              
	 double	FollowPathWeight;         

	 
	 double m_dWaypointSeekDistSq;

public:
	//pointer to any current path
	Path*          m_pPath;
	Point PointToWorldSpace(const Point &point,
		const Point &AgentHeading,
		const Point &AgentSide,
		const Point &AgentPosition)
	{
		//make a copy of the point
		Point TransPoint = point;

		//create a transformation matrix
		C2DMatrix matTransform;

		//rotate
		matTransform.Rotate(AgentHeading, AgentSide);

		//and translate
		matTransform.Translate(AgentPosition.x, AgentPosition.y);

		//now transform the vertices
		matTransform.TransformVector2Ds(TransPoint);

		return TransPoint;
	}
	//--------------------- PointToLocalSpace --------------------------------
	//
	//------------------------------------------------------------------------
 Point PointToLocalSpace(const Point &point,
		Point &AgentHeading,
		Point &AgentSide,
		Point &AgentPosition)
	{

		//make a copy of the point
		Point TransPoint = point;

		//create a transformation matrix
		C2DMatrix matTransform;

		double Tx = -AgentPosition.dot(AgentHeading);
		double Ty = -AgentPosition.dot(AgentSide);

		//create the transformation matrix
		matTransform._11(AgentHeading.x); matTransform._12(AgentSide.x);
		matTransform._21(AgentHeading.y); matTransform._22(AgentSide.y);
		matTransform._31(Tx);           matTransform._32(Ty);

		//now transform the vertices
		matTransform.TransformVector2Ds(TransPoint);

		return TransPoint;
	}

 //--------------------- VectorToWorldSpace --------------------------------
 //
 //  Transforms a vector from the agent's local space into world space
 //------------------------------------------------------------------------
 inline Point VectorToWorldSpace(const Point &vec,
	 const Point &AgentHeading,
	 const Point &AgentSide)
 {
	 //make a copy of the point
	 Point TransVec = vec;

	 //create a transformation matrix
	 C2DMatrix matTransform;

	 //rotate
	 matTransform.Rotate(AgentHeading, AgentSide);

	 //now transform the vertices
	 matTransform.TransformVector2Ds(TransVec);

	 return TransVec;
 }

 //-------------------------- Vec2DRotateAroundOrigin --------------------------
 //
 //  rotates a vector ang rads around the origin
 //-----------------------------------------------------------------------------
 void Vec2DRotateAroundOrigin(Point& v, double ang)
 {
	 //create a transformation matrix
	 C2DMatrix mat;

	 //rotate
	 mat.Rotate(ang);

	 //now transform the object's vertices
	 mat.TransformVector2Ds(v);
 }

 //-------------------- LineIntersection2D-------------------------
 //
 //	Given 2 lines in 2D space AB, CD this returns true if an 
 //	intersection occurs and sets dist to the distance the intersection
 //  occurs along AB. Also sets the 2d vector point to the point of
 //  intersection
 //----------------------------------------------------------------- 
bool LineIntersection2D(Point   A,
	 Point   B,
	 Point   C, 
	 Point   D,
	 double&     dist,
	 Point&  point)
 {

	 double rTop = (A.y-C.y)*(D.x-C.x)-(A.x-C.x)*(D.y-C.y);
	 double rBot = (B.x-A.x)*(D.y-C.y)-(B.y-A.y)*(D.x-C.x);

	 double sTop = (A.y-C.y)*(B.x-A.x)-(A.x-C.x)*(B.y-A.y);
	 double sBot = (B.x-A.x)*(D.y-C.y)-(B.y-A.y)*(D.x-C.x);

	 if ( (rBot == 0) || (sBot == 0))
	 {
		 //lines are parallel
		 return false;
	 }

	 double r = rTop/rBot;
	 double s = sTop/sBot;

	 if( (r > 0) && (r < 1) && (s > 0) && (s < 1) )
	 {
		 dist = ccpDistance(A,B) * r;

		 point = A + r * (B - A);

		 return true;
	 }

	 else
	 {
		 dist = 0;

		 return false;
	 }
 }

	//Arrive makes use of these to determine how quickly a vehicle
	//should decelerate to its target
	enum Deceleration{slow = 3, normal = 2, fast = 1};

	//default
	Deceleration m_Deceleration;

	SteeringBehavior();
	~SteeringBehavior();
	static SteeringBehavior* createWithTarget(Vehicle* pVehicle);
	bool initWithTarget(Vehicle* pVehicle);
	Point Calculate();

	//this behavior moves the agent towards a target position
	Point Seek(Point TargetPos);
	void seekOn(){m_seekOn = true;}
	void seekOff(){m_seekOn = false;}
	//this behavior returns a vector that moves the agent away
	//from a target position
	Point Flee(Point TargetPos);
	void fleeOn(){m_fleeOn = true;}
	void fleeOff(){m_fleeOn = false;}
	//this behavior is similar to seek but it attempts to arrive 
	//at the target position with a zero velocity
	Point Arrive(Point     TargetPos,
		Deceleration deceleration);
	void arriveOn(){m_arriveOn = true;}
	void arriveOff(){m_arriveOn = false;}
	//this behavior predicts where an agent will be in time T and seeks
	//towards that point to intercept it.
	Point Pursuit(const Vehicle* agent);
	void pursuitOn(){m_pursuitOn = true;}
	void pursuitOff(){m_pursuitOn = false;}
	//this behavior attempts to evade a pursuer
	Point Evade(const Vehicle* agent);
	void evadeOn(){m_evadeOn = true;}
	void evadeOff(){m_evadeOn = false;}
	//this behavior makes the agent wander about randomly
	Point Wander();
	void wanderOn(){m_wanderOn = true;}
	void wanderOff(){m_wanderOn = false;}
	//this returns a steering force which will attempt to keep the agent 
	//away from any obstacles it may encounter
	Point ObstacleAvoidance(const std::vector<Obstacle*>& obstacles);
	void obstacleAvoidanceOn(){m_obstacleAvoidanceOn = true;}
	void obstacleAvoidanceOff(){m_obstacleAvoidanceOn = false;}
	//given another agent position to hide from and a list of BaseGameEntitys this
	//method attempts to put an obstacle between itself and its opponent
	Vehicle* pHunter;
	Point Hide(const Vehicle* hunter, const std::vector<Obstacle*>& obstacles);
	void hideOn(){m_hideOn = true;}
	void hideOff(){m_hideOn = false;}
	//helper method for Hide. Returns a position located on the other
	//side of an obstacle to the pursuer
	Point GetHidingPosition(const Point& posOb,
		const double     radiusOb,
		const Point& posHunter);
	//this returns a steering force which will keep the agent away from any
	//walls it may encounter
	Point WallAvoidance(const std::vector<Wall2D> &walls);
	void  wallAvoidanceOn(){m_wallAvoidanceOn = true;}
	void  wallAvoidanceOff(){m_wallAvoidanceOn = false;}
	//creates the antenna utilized by the wall avoidance behavior
	void      CreateFeelers();

	//this results in a steering force that attempts to steer the vehicle
	//to the center of the vector connecting two moving agents.
	Point Interpose(const Vehicle* VehicleA, const Vehicle* VehicleB);
	void interposeOn(){m_interposeOn = true;}
	void interposeOff(){m_interposeOn = false;}

	//given a series of Vector2Ds, this method produces a force that will
	//move the agent along the waypoints in order
	Point FollowPath();
	void followPathOn(){m_followPathOn = true;}
	void followPathOff(){m_followPathOn = false;}
};

