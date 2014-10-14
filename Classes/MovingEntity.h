#pragma once
#include "BaseGameEntity.h"
#include "cocos2d.h"
USING_NS_CC;
class MovingEntity :
	public BaseGameEntity
{
public:
	MovingEntity(void);
	virtual ~MovingEntity(void);

	static MovingEntity* createWithInfo(Point position,
		double    radius,
		Point velocity,
		double    max_speed,
		Point heading,
		double    mass,
		double    max_turn_rate,
		double    max_force);
	
	bool initWithInfo(Point position,
		double    radius,
		Point velocity,
		double    max_speed,
		Point heading,
		double    mass,
		double    max_turn_rate,
		double    max_force);

	Point  Velocity()const{return m_vVelocity;}
	void      SetVelocity(const Point& NewVel){m_vVelocity = NewVel;}

	double     Mass()const{return m_dMass;}

	Point  Side()const{return m_vSide;}

	double     MaxSpeed()const{return m_dMaxSpeed;}                       
	void      SetMaxSpeed(double new_speed){m_dMaxSpeed = new_speed;}

	double     MaxForce()const{return m_dMaxForce;}
	void      SetMaxForce(double mf){m_dMaxForce = mf;}
	bool      IsSpeedMaxedOut()const{return m_dMaxSpeed*m_dMaxSpeed >= ccpLengthSQ(m_vVelocity);}
	double     Speed()const{return ccpLength(m_vVelocity);}
	double     SpeedSq()const{return ccpLengthSQ(m_vVelocity);}

	Point  Heading()const{return m_vHeading;}
	void      SetHeading(Point new_heading);
	bool      RotateHeadingToFacePosition(Point target);

	double     MaxTurnRate()const{return m_dMaxTurnRate;}
	void      SetMaxTurnRate(double val){m_dMaxTurnRate = val;}

	enum {clockwise = 1, anticlockwise = -1};
	int ccpSign(Point v1,Point v2);

protected:
	Point    m_vVelocity;

	//a normalized vector pointing in the direction the entity is heading. 
	Point    m_vHeading;

	//a vector perpendicular to the heading vector
	Point    m_vSide; 

	double       m_dMass;

	//the maximum speed this entity may travel at.
	double       m_dMaxSpeed;

	//the maximum force this entity can produce to power itself 
	//(think rockets and thrust)
	double        m_dMaxForce;

	//the maximum rate (radians per second)this vehicle can rotate         
	double       m_dMaxTurnRate;
};

