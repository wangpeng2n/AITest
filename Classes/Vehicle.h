#pragma once
#include "MovingEntity.h"

class SteeringBehavior;
class Vehicle : public MovingEntity
{
public:
	Vehicle();
	~Vehicle();
	
	static Vehicle* createWithInfo(Point position,
		double    rotation,
		Point	  velocity,
		double    mass,
		double    max_force,
		double    max_speed,
		double    max_turn_rate,
		double	  radius,
		Sprite* sprite);

	bool initWithInfo(Point position,
		double    rotation,
		Point	  velocity,
		double    mass,
		double    max_force,
		double    max_speed,
		double    max_turn_rate,
		double	  radius,
		Sprite* sprite);

	SteeringBehavior*const  Steering()const{return m_pSteering;}
	Sprite* getSprite() const{return m_sprite;};

	void update(float delta);
	float timeDelta;
	Point Truncate(double max,Point p)
	{
		if (ccpLength(p) > max)
		{
			Point temp = ccpNormalize(p);
			temp *= max;
			return temp;
		} else{
			return p;
		}
	}

	//treats a window as a toroid
	inline void WrapAround(Point &pos, int MaxX, int MaxY)
	{
		if (pos.x > MaxX) {pos.x = 0.0;}

		if (pos.x < 0)    {pos.x = (double)MaxX;}

		if (pos.y < 0)    {pos.y = (double)MaxY;}

		if (pos.y > MaxY) {pos.y = 0.0;}
	}

private:
	SteeringBehavior* m_pSteering;
	Sprite* m_sprite;
};

