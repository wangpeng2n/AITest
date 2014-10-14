#include "SteeringBehavior.h"


SteeringBehavior::SteeringBehavior()
{
}

SteeringBehavior::~SteeringBehavior()
{
}

SteeringBehavior* SteeringBehavior::createWithTarget(Vehicle* pVehicle)
{
	SteeringBehavior *pRet = new SteeringBehavior(); 
	if (pRet && pRet->initWithTarget(pVehicle)) 
	{ 
		pRet->autorelease(); 
		return pRet; 
	} 
	else 
	{ 
		delete pRet; 
		pRet = NULL; 
		return NULL; 
	} 
}

bool SteeringBehavior::initWithTarget(Vehicle* pVehicle)
{
	m_pVehicle = pVehicle;

	m_seekForce = Point::ZERO;

	return true;
}

Point SteeringBehavior::Calculate()
{
	//reset the steering force
	m_vSteeringForce = Point::ZERO;
	m_vSteeringForce+= m_seekForce;

	return m_vSteeringForce;
}

Point SteeringBehavior::Seek(Point TargetPos)
{
	Point DesiredVelocity = ccpNormalize(TargetPos - m_pVehicle->getPosition())
		* m_pVehicle->MaxSpeed();
	m_seekForce = (DesiredVelocity - m_pVehicle->Velocity());
	return (DesiredVelocity - m_pVehicle->Velocity());
}