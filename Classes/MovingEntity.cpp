#include "MovingEntity.h"
#include "C2DMatrix.h"

MovingEntity::MovingEntity(void)
{
}


MovingEntity::~MovingEntity(void)
{
}


MovingEntity* MovingEntity::createWithInfo(Point position,
							 double    radius,
							 Point velocity,
							 double    max_speed,
							 Point heading,
							 double    mass,
							 double    max_turn_rate,
							 double    max_force)
{
	MovingEntity *pRet = new MovingEntity(); 
		if (pRet && pRet->initWithInfo(position,radius,velocity,max_speed,heading,mass,max_turn_rate,max_force)) 
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

bool MovingEntity::initWithInfo(Point position,
				  double    radius,
				  Point velocity,
				  double    max_speed,
				  Point heading,
				  double    mass,
				  double    max_turn_rate,
				  double    max_force)
{
	if (!BaseGameEntity::init())
	{
		return false;
	}
	this->setPosition(position);
	this->setContentSize(Size(radius*2,radius*2));
	m_vHeading = heading;
	m_vVelocity = velocity;
	m_dMass = mass;
	m_vSide = ccpPerp(heading);
	m_dMaxSpeed = max_speed;
	m_dMaxTurnRate = max_turn_rate;
	m_dMaxForce = max_force;
	return true;
}


//--------------------------- RotateHeadingToFacePosition ---------------------
//
//  given a target position, this method rotates the entity's heading and
//  side vectors by an amount not greater than m_dMaxTurnRate until it
//  directly faces the target.
//
//  returns true when the heading is facing in the desired direction
//-----------------------------------------------------------------------------
inline bool MovingEntity::RotateHeadingToFacePosition(Point target)
{
	Point toTarget = ccpNormalize(target - this->getPosition());

	//first determine the angle between the heading vector and the target
	double angle = acos(ccpDot(m_vHeading,toTarget));

	//return true if the player is facing the target
	if (angle < 0.00001) return true;

	//clamp the amount to turn to the max turn rate
	if (angle > m_dMaxTurnRate) angle = m_dMaxTurnRate;

	//The next few lines use a rotation matrix to rotate the player's heading
	//vector accordingly
	C2DMatrix RotationMatrix;

	//notice how the direction of rotation has to be determined when creating
	//the rotation matrix
	RotationMatrix.Rotate(angle *ccpSign(m_vHeading,toTarget));	
	RotationMatrix.TransformVector2Ds(m_vHeading);
	RotationMatrix.TransformVector2Ds(m_vVelocity);
	//finally recreate m_vSide
	m_vSide = ccpPerp(m_vHeading);

	return false;
}


//------------------------- SetHeading ----------------------------------------
//
//  first checks that the given heading is not a vector of zero length. If the
//  new heading is valid this fumction sets the entity's heading and side 
//  vectors accordingly
//-----------------------------------------------------------------------------
inline void MovingEntity::SetHeading(Point new_heading)
{
	assert( (ccpLengthSQ(new_heading) - 1.0) < 0.00001);

	m_vHeading = new_heading;

	//the side vector must always be perpendicular to the heading
	m_vSide = ccpPerp(m_vHeading);
}

inline int MovingEntity::ccpSign(Point v1,Point v2)
{
	if (v1.y*v2.x > v1.x*v2.y)
	{ 
		return anticlockwise;
	}
	else 
	{
		return clockwise;
	}
}