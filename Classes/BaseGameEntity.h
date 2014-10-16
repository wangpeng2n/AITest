#pragma once
#include "cocos2d.h"

class BaseGameEntity :
	public cocos2d::Node
{
public:
	BaseGameEntity(void);
	virtual ~BaseGameEntity(void);
	//--------------------- PointToWorldSpace --------------------------------
	//
	//  Transforms a point from the agent's local space into world space
	//------------------------------------------------------------------------
	
};

