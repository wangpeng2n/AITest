#include "Vehicle.h"
#include "SteeringBehavior.h"

Vehicle::Vehicle()
{
}

Vehicle::~Vehicle()
{
}

Vehicle* Vehicle::createWithInfo(Point position,
							   double    rotation,
							   Point	  velocity,
							   double    mass,
							   double    max_force,
							   double    max_speed,
							   double    max_turn_rate,
							   double	  radius,
							   Sprite* sprite)
{
	Vehicle *pRet = new Vehicle(); 
	if (pRet && pRet->initWithInfo(position,
		rotation,
		velocity,
		mass,
		max_force,
		max_speed,
		max_turn_rate,radius,sprite)) 
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

bool Vehicle::initWithInfo(Point position,
				  double    rotation,
				  Point	  velocity,
				  double    mass,
				  double    max_force,
				  double    max_speed,
				  double    max_turn_rate,
				  double	  radius,
				  Sprite* sprite)
{
	if (!MovingEntity::initWithInfo(position,radius,velocity,max_speed,
		Point(sin(rotation),-cos(rotation)),mass,max_turn_rate,max_force))
	{
		return false;
	}
	m_pSteering = SteeringBehavior::createWithTarget(this);
	m_sprite = sprite;
	this->addChild(m_sprite);
	return true;
}

void Vehicle::update(float delta)
{

	//keep a record of its old position so we can update its cell later
	//in this method
	Point OldPos = this->getPosition();


	Point SteeringForce;

	//calculate the combined force from each steering behavior in the 
	//vehicle's list
	SteeringForce = m_pSteering->Calculate();

	//Acceleration = Force/Mass
	Point acceleration = SteeringForce / m_dMass;

	//update velocity
	m_vVelocity += acceleration * delta; 

	//make sure vehicle does not exceed maximum velocity
	
	m_vVelocity = this->Truncate(m_dMaxSpeed,m_vVelocity);

	//update the position
	this->setPosition(this->getPosition()+ (m_vVelocity *delta));

	//update the heading if the vehicle has a non zero velocity
	if (ccpLengthSQ(m_vVelocity) > 0.00000001)
	{    
		m_vHeading = ccpNormalize(m_vVelocity);

		m_vSide = ccpPerp(m_vHeading);
	}

	//EnforceNonPenetrationConstraint(this, World()->Agents());

	//treat the screen as a toroid
	Point tempP = this->getPosition();
	WrapAround(tempP, Director::getInstance()->getVisibleSize().width,  Director::getInstance()->getVisibleSize().height);
	this->setPosition(tempP);

	return;
}