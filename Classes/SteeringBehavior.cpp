
//NumAgents                300
//
//	NumObstacles             7
//	MinObstacleRadius        10
//	MaxObstacleRadius        30
//
//
//
//	//number of horizontal cells used for spatial partitioning
//	NumCellsX                7
//	//number of vertical cells used for spatial partitioning
//	NumCellsY                7
//
//
//	//how many samples the smoother will use to average a value
//	NumSamplesForSmoothing   10
//
//
//	//this is used to multiply the steering force AND all the multipliers
//	//found in SteeringBehavior
//	SteeringForceTweaker     200.0
//
//	SteeringForce            2.0
//	MaxSpeed                 150.0
//	VehicleMass              1.0
//	VehicleScale             3.0
//
//	//use these values to tweak the amount that each steering force
//	//contributes to the total steering force
//	SeparationWeight          1.0
//	AlignmentWeight           1.0
//	CohesionWeight            2.0
//	ObstacleAvoidanceWeight   10.0
//	WallAvoidanceWeight       10.0
//	WanderWeight              1.0
//	SeekWeight                1.0
//	FleeWeight                1.0
//	ArriveWeight              1.0
//	PursuitWeight             1.0
//	OffsetPursuitWeight       1.0
//	InterposeWeight           1.0
//	HideWeight                1.0
//	EvadeWeight               0.01
//	FollowPathWeight          0.05
//
//	//how close a neighbour must be before an agent perceives it (considers it
//	//to be within its neighborhood)
//	ViewDistance              50.0
//
//	//used in obstacle avoidance
//	MinDetectionBoxLength     40.0
//
//	//used in wall avoidance
//	WallDetectionFeelerLength 40.0
//
//	//these are the probabilities that a steering behavior will be used
//	//when the Prioritized Dither calculate method is used to sum
//	//combined behaviors
//	float prWallAvoidance       0.5
//	prObstacleAvoidance         0.5
//	prSeparation                0.2
//	prAlignment                 0.3
//	prCohesion                  0.6
//	prWander                    0.8
//	prSeek                      0.8
//	prFlee                      0.6
//	prEvade                     1.0
//	prHide                      0.8
//	prArrive                    0.5

#include "SteeringBehavior.h"
#include "HelloWorldScene.h"

SteeringBehavior::SteeringBehavior()
{
}

SteeringBehavior::~SteeringBehavior()
{
	delete m_pPath;
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

	m_dWallDetectionFeelerLength = 100;
	m_Feelers.push_back(Point::ZERO);
	m_Feelers.push_back(Point::ZERO);
	m_Feelers.push_back(Point::ZERO);

	 m_seekOn = false;
	 m_hideOn=false;
	 m_obstacleAvoidanceOn=false;
	 m_wanderOn=false;
	 m_evadeOn=false;
	 m_pursuitOn=false;
	 m_arriveOn=false;
	 m_fleeOn=false;
	 m_interposeOn=false;
	 m_wallAvoidanceOn=false;
	 m_followPathOn = false;
	//use these values to tweak the amount that each steering force
	//contributes to the total steering force
	SeparationWeight    =      1.0;
	AlignmentWeight     =      1.0;
	CohesionWeight       =     2.0;
	ObstacleAvoidanceWeight =  10.0;
	WallAvoidanceWeight    =   10.0;
	WanderWeight        =      1.0;
	SeekWeight        =        1.0;
	FleeWeight        =        1.0;
	ArriveWeight      =        1.0;
	PursuitWeight      =       1.0;
	OffsetPursuitWeight  =     1.0;
	InterposeWeight     =      1.0;
	HideWeight         =       1.0;
	EvadeWeight        =       0.01;
	FollowPathWeight    =      1;

	//create a Path
	m_pPath = new Path(5,
		100,
		50,
		860,
		590,
		true);

	m_dWaypointSeekDistSq = 100;

	return true;
}

Point SteeringBehavior::Calculate()
{
	//reset the steering force
	m_vSteeringForce = Point::ZERO;

	HelloWorld* gameWorld = dynamic_cast<HelloWorld*>(this->m_pVehicle->getParent());

	Point force;

	if (m_obstacleAvoidanceOn)
	{
		force = ObstacleAvoidance(gameWorld->Obstacles()) * 
			ObstacleAvoidanceWeight;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}
	if (m_evadeOn)
	{
		/*assert(m_pTargetAgent1 && "Evade target not assigned");

		force = Evade(m_pTargetAgent1) * m_dWeightEvade;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;*/
	}


	if (m_fleeOn)
	{
		force = Flee(gameWorld->Crosshair()) * FleeWeight;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}

	if (m_seekOn)
	{
		force = Seek(gameWorld->Crosshair()) * SeekWeight;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}


	if (m_arriveOn)
	{
		force = Arrive(gameWorld->Crosshair(), fast) * ArriveWeight;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}
	if (m_wanderOn)
	{
		force = Wander() * WanderWeight;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}
	if (m_hideOn)
	{
		CCAssert(pHunter!=NULL,"no set hide hunter!");
		force = Hide(pHunter,gameWorld->Obstacles()) * WanderWeight;
		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}
	if (m_followPathOn)
	{
		force = FollowPath() * FollowPathWeight;

		if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
	}

	return  m_vSteeringForce;
}

//--------------------- AccumulateForce ----------------------------------
//
//  This function calculates how much of its max steering force the 
//  vehicle has left to apply and then applies that amount of the
//  force to add.
//------------------------------------------------------------------------
bool SteeringBehavior::AccumulateForce(Point &RunningTot,
									   Point ForceToAdd)
{

	//calculate how much steering force the vehicle has used so far
	double MagnitudeSoFar = RunningTot.length();

	//calculate how much steering force remains to be used by this vehicle
	double MagnitudeRemaining = m_pVehicle->MaxForce() - MagnitudeSoFar;

	//return false if there is no more force left to use
	if (MagnitudeRemaining <= 0.0) return false;

	//calculate the magnitude of the force we want to add
	double MagnitudeToAdd = ForceToAdd.length();

	//if the magnitude of the sum of ForceToAdd and the running total
	//does not exceed the maximum force available to this vehicle, just
	//add together. Otherwise add as much of the ForceToAdd vector is
	//possible without going over the max.
	if (MagnitudeToAdd < MagnitudeRemaining)
	{
		RunningTot += ForceToAdd;
	}

	else
	{
		//add it to the steering force
		RunningTot += (ccpNormalize(ForceToAdd) * MagnitudeRemaining); 
	}

	return true;
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
		seekOn();
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

//--------------------------- Wander -------------------------------------
//
//  This behavior makes the agent wander about randomly
//------------------------------------------------------------------------
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

//---------------------- ObstacleAvoidance -------------------------------
//
//  Given a vector of CObstacles, this method returns a steering force
//  that will prevent the agent colliding with the closest obstacle
//------------------------------------------------------------------------
Point SteeringBehavior::ObstacleAvoidance(const std::vector<Obstacle*>& obstacles)
{
	//the detection box length is proportional to the agent's velocity
	m_dDBoxLength = 0;
	float MinDetectionBoxLength = 150;
	m_dDBoxLength = MinDetectionBoxLength + 
		(m_pVehicle->Speed()/m_pVehicle->MaxSpeed()) *
		MinDetectionBoxLength;

	////tag all obstacles within range of the box for processing
	//m_pVehicle->World()->TagObstaclesWithinViewRange(m_pVehicle, m_dDBoxLength);

	//this will keep track of the closest intersecting obstacle (CIB)
	Obstacle* ClosestIntersectingObstacle = NULL;

	//this will be used to track the distance to the CIB
	double DistToClosestIP = MaxDouble;

	//this will record the transformed local coordinates of the CIB
	Point LocalPosOfClosestObstacle;

	std::vector<Obstacle*>::const_iterator curOb = obstacles.begin();

	while(curOb != obstacles.end())
	{
		if ((*curOb)->position.distance(m_pVehicle->getPosition())<300)
		{
			//if the obstacle has been tagged within range proceed
			//calculate this obstacle's position in local space
			Point LocalPos = PointToLocalSpace((*curOb)->position,
				m_pVehicle->Heading(),
				m_pVehicle->Side(),
				(Point)m_pVehicle->getPosition());

			//if the local position has a negative x value then it must lay
			//behind the agent. (in which case it can be ignored)
			if (LocalPos.x >= 0)
			{
				//if the distance from the x axis to the object's position is less
				//than its radius + half the width of the detection box then there
				//is a potential intersection.
				double ExpandedRadius = (*curOb)->radius + m_pVehicle->getContentSize().width/2.0;

				if (fabs(LocalPos.y) < ExpandedRadius)
				{
					//now to do a line/circle intersection test. The center of the 
					//circle is represented by (cX, cY). The intersection points are 
					//given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
					//We only need to look at the smallest positive value of x because
					//that will be the closest point of intersection.
					double cX = LocalPos.x;
					double cY = LocalPos.y;

					//we only need to calculate the sqrt part of the above equation once
					double SqrtPart = sqrt(ExpandedRadius*ExpandedRadius - cY*cY);

					double ip = cX - SqrtPart;

					if (ip <= 0.0)
					{
						ip = cX + SqrtPart;
					}

					//test to see if this is the closest so far. If it is keep a
					//record of the obstacle and its local coordinates
					if (ip < DistToClosestIP)
					{
						DistToClosestIP = ip;

						ClosestIntersectingObstacle = *curOb;

						LocalPosOfClosestObstacle = LocalPos;
					}         
				}
			}
		}
		++curOb;
	}

	//if we have found an intersecting obstacle, calculate a steering 
	//force away from it
	Point SteeringForce;
	if (ClosestIntersectingObstacle)
	{
		//the closer the agent is to an object, the stronger the 
		//steering force should be
		double multiplier = 1.0 + (m_dDBoxLength - LocalPosOfClosestObstacle.x) /
			m_dDBoxLength;

		//calculate the lateral force
		SteeringForce.y = (ClosestIntersectingObstacle->radius-
			LocalPosOfClosestObstacle.y)  * multiplier;   

		//apply a braking force proportional to the obstacles distance from
		//the vehicle. 
		const double BrakingWeight = 0.5;

		SteeringForce.x = (ClosestIntersectingObstacle->radius - 
			LocalPosOfClosestObstacle.x) * 
			BrakingWeight;
	}

	//finally, convert the steering vector from local to world space
	Point temp = VectorToWorldSpace(SteeringForce,
		m_pVehicle->Heading(),
		m_pVehicle->Side());
	return temp;
}
//------------------------------- CreateFeelers --------------------------
//
//  Creates the antenna utilized by WallAvoidance
//------------------------------------------------------------------------
void SteeringBehavior::CreateFeelers()
{
	//feeler pointing straight in front

	m_Feelers[0] = (m_pVehicle->getPosition() + m_dWallDetectionFeelerLength * m_pVehicle->Heading());

	//feeler to left
	Point temp = m_pVehicle->Heading();

	Vec2DRotateAroundOrigin(temp, HalfPi * 3.5f);
	m_Feelers[1] = (m_pVehicle->getPosition() + m_dWallDetectionFeelerLength/2.0f * temp);

	//feeler to right
	temp = m_pVehicle->Heading();
	Vec2DRotateAroundOrigin(temp, HalfPi * 0.5f);
	m_Feelers[2] = (m_pVehicle->getPosition() + m_dWallDetectionFeelerLength/2.0f * temp);
}
//--------------------------- WallAvoidance --------------------------------
//
//  This returns a steering force that will keep the agent away from any
//  walls it may encounter
//------------------------------------------------------------------------
Point SteeringBehavior::WallAvoidance(const std::vector<Wall2D>& walls)
{
	//the feelers are contained in a std::vector, m_Feelers
	CreateFeelers();

	double DistToThisIP    = 0.0;
	double DistToClosestIP = MaxDouble;

	//this will hold an index into the vector of walls
	int ClosestWall = -1;

	Point SteeringForce,
		point,         //used for storing temporary info
		ClosestPoint;  //holds the closest intersection point

	//examine each feeler in turn
	for (unsigned int flr=0; flr<m_Feelers.size(); ++flr)
	{
		//run through each wall checking for any intersection points
		for (unsigned int w=0; w<walls.size(); ++w)
		{
			if (LineIntersection2D(m_pVehicle->getPosition(),
				m_Feelers[flr],
				walls[w].startPos,
				walls[w].endPos,
				DistToThisIP,
				point))
			{
				//is this the closest found so far? If so keep a record
				if (DistToThisIP < DistToClosestIP)
				{
					DistToClosestIP = DistToThisIP;

					ClosestWall = w;

					ClosestPoint = point;
				}
			}
		}//next wall


		//if an intersection point has been detected, calculate a force  
		//that will direct the agent away
		if (ClosestWall >=0)
		{
			//calculate by what distance the projected position of the agent
			//will overshoot the wall
			Point OverShoot = m_Feelers[flr] - ClosestPoint;

			//create a force in the direction of the wall normal, with a 
			//magnitude of the overshoot
			SteeringForce = walls[ClosestWall].forceDirect * OverShoot.length();
		}

	}//next feeler

	return SteeringForce;
}

//--------------------------- Interpose ----------------------------------
//
//  Given two agents, this method returns a force that attempts to 
//  position the vehicle between them
//------------------------------------------------------------------------
Point SteeringBehavior::Interpose(const Vehicle* AgentA,
								  const Vehicle* AgentB)
{
	//first we need to figure out where the two agents are going to be at 
	//time T in the future. This is approximated by determining the time
	//taken to reach the mid way point at the current time at at max speed.
	Point MidPoint = (AgentA->getPosition() + AgentB->getPosition()) / 2.0;

	double TimeToReachMidPoint = ccpDistance(m_pVehicle->getPosition(), MidPoint) /
		m_pVehicle->MaxSpeed();

	//now we have T, we assume that agent A and agent B will continue on a
	//straight trajectory and extrapolate to get their future positions
	Point APos = AgentA->getPosition() + AgentA->Velocity() * TimeToReachMidPoint;
	Point BPos = AgentB->getPosition() + AgentB->Velocity() * TimeToReachMidPoint;

	//calculate the mid point of these predicted positions
	MidPoint = (APos + BPos) / 2.0;

	//then steer to Arrive at it

	return Arrive(MidPoint, fast);
}

//--------------------------- Hide ---------------------------------------
//
//------------------------------------------------------------------------
Point SteeringBehavior::Hide(const Vehicle* hunter, const std::vector<Obstacle*>& obstacles)
{
	double    DistToClosest = MaxDouble;
	Point BestHidingSpot;

	std::vector<Obstacle*>::const_iterator curOb = obstacles.begin();
	std::vector<Obstacle*>::const_iterator closest;

	while(curOb != obstacles.end())
	{
		//calculate the position of the hiding spot for this obstacle
		Point HidingSpot = GetHidingPosition((*curOb)->position,
			(*curOb)->radius,
			hunter->getPosition());

		//work in distance-squared space to find the closest hiding
		//spot to the agent
		double dist = ccpDistanceSQ(HidingSpot, m_pVehicle->getPosition());

		if (dist < DistToClosest)
		{
			DistToClosest = dist;

			BestHidingSpot = HidingSpot;

			closest = curOb;
		}  

		++curOb;

	}//end while

	//if no suitable obstacles found then Evade the hunter
	if (DistToClosest == MaxFloat)
	{
		
		return Evade(hunter);
	}

	//else use Arrive on the hiding spot

	return Arrive(BestHidingSpot, fast);
}

//------------------------- GetHidingPosition ----------------------------
//
//  Given the position of a hunter, and the position and radius of
//  an obstacle, this method calculates a position DistanceFromBoundary 
//  away from its bounding radius and directly opposite the hunter
//------------------------------------------------------------------------
Point SteeringBehavior::GetHidingPosition(const Point& posOb,
										  const double     radiusOb,
										  const Point& posHunter)
{
	//calculate how far away the agent is to be from the chosen obstacle's
	//bounding radius
	const double DistanceFromBoundary = 30.0;
	double       DistAway    = radiusOb + DistanceFromBoundary;

	//calculate the heading toward the object from the hunter
	Point ToOb = ccpNormalize(posOb - posHunter);

	//scale it to size and add to the obstacles position to get
	//the hiding spot.
	return (ToOb * DistAway) + posOb;
}

//------------------------------- FollowPath -----------------------------
//
//  Given a series of Vector2Ds, this method produces a force that will
//  move the agent along the waypoints in order. The agent uses the
// 'Seek' behavior to move to the next waypoint - unless it is the last
//  waypoint, in which case it 'Arrives'
//------------------------------------------------------------------------
Point SteeringBehavior::FollowPath()
{ 
	//move to next target if close enough to current target (working in
	//distance squared space)
	if(ccpDistanceSQ(m_pPath->CurrentWaypoint(), m_pVehicle->getPosition()) <
		m_dWaypointSeekDistSq)
	{
		m_pPath->SetNextWaypoint();
	}

	if (!m_pPath->Finished())
	{
	
		//return Seek(m_pPath->CurrentWaypoint());
		return Arrive(m_pPath->CurrentWaypoint(), fast);
	}

	else
	{
	
		return Arrive(m_pPath->CurrentWaypoint(), fast);
	}
}