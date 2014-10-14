#pragma once
#include "cocos2d.h"
#include "Vehicle.h"

USING_NS_CC;
class Vehicle;
class SteeringBehavior : public Ref
{
public:
	SteeringBehavior();
	~SteeringBehavior();
	static SteeringBehavior* createWithTarget(Vehicle* pVehicle);
	bool initWithTarget(Vehicle* pVehicle);
	Point Calculate();

	//this behavior moves the agent towards a target position
	Point Seek(Point TargetPos);
private:
	//a pointer to the owner of this instance
	Vehicle*     m_pVehicle;   

	//the steering force created by the combined effect of all
	//the selected behaviors
	Point    m_vSteeringForce;

	Point	 m_seekForce;

};
