#pragma once
#include "cocos2d.h"
#include "Vehicle.h"
#include "C2DMatrix.h"

USING_NS_CC;
class Vehicle;

//--------------------------- Constants ----------------------------------

//the radius of the constraining circle for the wander behavior
const double WanderRad    = 1.2;
//distance the wander circle is projected in front of the agent
const double WanderDist   = 1.8;
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



public:
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
	//this behavior returns a vector that moves the agent away
	//from a target position
	Point Flee(Point TargetPos);
	//this behavior is similar to seek but it attempts to arrive 
	//at the target position with a zero velocity
	Point Arrive(Point     TargetPos,
		Deceleration deceleration);
	//this behavior predicts where an agent will be in time T and seeks
	//towards that point to intercept it.
	Point Pursuit(const Vehicle* agent);
	//this behavior attempts to evade a pursuer
	Point Evade(const Vehicle* agent);
	//this behavior makes the agent wander about randomly
	Point Wander();
};
