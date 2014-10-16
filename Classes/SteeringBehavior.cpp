#include "SteeringBehavior.h"
#include "HelloWorldScene.h"

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

	m_dWanderDistance = WanderDist*pVehicle->getSprite()->getContentSize().width;
	m_dWanderJitter = WanderJitterPerSec;
	m_dWanderRadius = WanderRad*pVehicle->getSprite()->getContentSize().width;

	//stuff for the wander behavior
	double theta = CCRANDOM_0_1() * TwoPi;

	//create a vector to a target position on the wander circle
	m_vWanderTarget = Point(m_dWanderRadius * cos(theta),
		m_dWanderRadius * sin(theta));

	return true;
}

Point SteeringBehavior::Calculate()
{
	//reset the steering force
	m_vSteeringForce = Point::ZERO;
	/*m_vSteeringForce+= m_seekForce;*/
	HelloWorld* gameWorld = dynamic_cast<HelloWorld*>(this->m_pVehicle->getParent());
	Point targetPos = gameWorld->Crosshair();
	return Wander();
}

Point SteeringBehavior::Seek(Point TargetPos)
{
	Point DesiredVelocity = ccpNormalize(TargetPos - m_pVehicle->getPosition())
		* m_pVehicle->MaxSpeed();

	return (DesiredVelocity - m_pVehicle->Velocity());
}

Point SteeringBehavior::Flee(Point TargetPos)
{
  //only flee if the target is within 'panic distance'. Work in distance
  //squared space.
 /* const double PanicDistanceSq = 100.0f * 100.0;
  if (Vec2DDistanceSq(m_pVehicle->Pos(), target) > PanicDistanceSq)
  {
    return Vector2D(0,0);
  }
  */

  Point DesiredVelocity = ccpNormalize(m_pVehicle->getPosition() - TargetPos) 
                            * m_pVehicle->MaxSpeed();
 
  return (DesiredVelocity - m_pVehicle->Velocity());
}

//--------------------------- Arrive -------------------------------------
//
//  This behavior is similar to seek but it attempts to arrive at the
//  target with a zero velocity
//------------------------------------------------------------------------
Point SteeringBehavior::Arrive(Point     TargetPos,
								  Deceleration deceleration)
{
	Point ToTarget = TargetPos - m_pVehicle->getPosition();

	//calculate the distance to the target
	double dist = ccpLength(ToTarget);

	if (dist > 0)
	{
		//because Deceleration is enumerated as an int, this value is required
		//to provide fine tweaking of the deceleration..
		const double DecelerationTweaker = 0.3;

		//calculate the speed required to reach the target given the desired
		//deceleration
		double speed =  dist / ((double)deceleration * DecelerationTweaker);     

		//make sure the velocity does not exceed the max
		speed = MIN(speed, m_pVehicle->MaxSpeed());

		//from here proceed just like Seek except we don't need to normalize 
		//the ToTarget vector because we have already gone to the trouble
		//of calculating its length: dist. 
		Point DesiredVelocity =  ToTarget * speed / dist;
		return (DesiredVelocity - m_pVehicle->Velocity());
	}

	return Point(0,0);
}
//------------------------------ Pursuit ---------------------------------
//
//  this behavior creates a force that steers the agent towards the 
//  evader
//------------------------------------------------------------------------
Point SteeringBehavior::Pursuit(const Vehicle* evader)
{
	//if the evader is ahead and facing the agent then we can just seek
	//for the evader's current position.
	Point ToEvader = evader->getPosition() - m_pVehicle->getPosition();
	
	double RelativeHeading = m_pVehicle->Heading().dot(evader->Heading());

	if ( (ToEvader.dot(m_pVehicle->Heading()) > 0) &&  
		(RelativeHeading < -0.95))  //acos(0.95)=18 degs
	{
		return Seek(evader->getPosition());
	}

	//Not considered ahead so we predict where the evader will be.

	//the lookahead time is propotional to the distance between the evader
	//and the pursuer; and is inversely proportional to the sum of the
	//agent's velocities
	double LookAheadTime = ToEvader.length() / 
		(m_pVehicle->MaxSpeed() + evader->Speed());

	//now seek to the predicted future position of the evader
	return Seek(evader->getPosition() + evader->Velocity() * LookAheadTime);
}

//----------------------------- Evade ------------------------------------
//
//  similar to pursuit except the agent Flees from the estimated future
//  position of the pursuer
//------------------------------------------------------------------------
Point SteeringBehavior::Evade(const Vehicle* pursuer)
{
	/* Not necessary to include the check for facing direction this time */

	Point ToPursuer = pursuer->getPosition() - m_pVehicle->getPosition();

	//uncomment the following two lines to have Evade only consider pursuers 
	//within a 'threat range'
	const double ThreatRange = 100.0;
	if (ToPursuer.lengthSquared() > ThreatRange * ThreatRange) return Point();

	//the lookahead time is propotional to the distance between the pursuer
	//and the pursuer; and is inversely proportional to the sum of the
	//agents' velocities
	double LookAheadTime = ToPursuer.length() / 
		(m_pVehicle->MaxSpeed() + pursuer->Speed());

	//now flee away from predicted future position of the pursuer
	return Flee(pursuer->getPosition() + pursuer->Velocity() * LookAheadTime);
}

Point SteeringBehavior::Wander()
{ 
	//this behavior is dependent on the update rate, so this line must
	//be included when using time independent framerate.
	double JitterThisTimeSlice = m_dWanderJitter * m_pVehicle->timeDelta;

	//first, add a small random vector to the target's position
	float randx = CCRANDOM_MINUS1_1();
	float randy = CCRANDOM_MINUS1_1();
	m_vWanderTarget += Point( randx* JitterThisTimeSlice,
		randy * JitterThisTimeSlice);

	//reproject this new vector back on to a unit circle
	m_vWanderTarget.normalize();

	//increase the length of the vector to the same as the radius
	//of the wander circle
	m_vWanderTarget *= m_dWanderRadius;

	//move the target into a position WanderDist in front of the agent
	Point target = m_vWanderTarget + Point(m_dWanderDistance, 0);
	
	//project the target into world space
	Point Target = PointToWorldSpace(target,
		m_pVehicle->Heading(),
		m_pVehicle->Side(), 
		m_pVehicle->getPosition());

	//and steer towards it
	/*Point wanderForce = Target - m_pVehicle->getPosition();*/
	return  Seek(Target);
}