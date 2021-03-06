#ifndef __HELLOWORLD_SCENE_H__
#define __HELLOWORLD_SCENE_H__

#include "cocos2d.h"
#include "BaseGameEntity.h"
#include "Vehicle.h"


typedef cocos2d::Vector<BaseGameEntity*>::iterator  ObIt;
typedef struct Obstacle
{
	cocos2d::Point position;
	float radius;
}Obstacle;
typedef struct Wall2D
{
	Point startPos;
	Point endPos;
	Point forceDirect;
}Wall2D;

class HelloWorld : public cocos2d::Layer
{
private:
	//a container of all the moving entities
	cocos2d::Vector<Vehicle*>         m_Vehicles;
	std::vector<Obstacle*>    m_Obstacles;
	std::vector<Wall2D>           m_Walls;
	//set true to pause the motion
	bool                          m_bPaused;

	//local copy of client window dimensions
	int                           m_cxClient,
								  m_cyClient;
	//the position of the crosshair
	Point                      m_vCrosshair;

	DrawNode*	m_drawNode;

	void CreateObstacles();
	void CreateWalls();

public:
    // there's no 'id' in cpp, so we recommend returning the class instance pointer
    static cocos2d::Scene* createScene();

    // Here's a difference. Method 'init' in cocos2d-x returns bool, instead of returning 'id' in cocos2d-iphone
    virtual bool init();  
    
    // implement the "static create()" method manually
    CREATE_FUNC(HelloWorld);

	const cocos2d::Vector<Vehicle*>&        Agents(){return m_Vehicles;}
	const std::vector<Obstacle*>& Obstacles(){return m_Obstacles;}
	const std::vector<Wall2D>&          Walls(){return m_Walls;}  

	void        TogglePause(){m_bPaused = !m_bPaused;}
	bool        Paused()const{return m_bPaused;}

	Point    Crosshair()const{return m_vCrosshair;}
	void        SetCrosshair(Point v){m_vCrosshair=v;}

	int   cxClient()const{return m_cxClient;}
	int   cyClient()const{return m_cyClient;}

	Vehicle* testVehicle;
	Vehicle* testVehicle2;
	void randMoveTo();
};

#endif // __HELLOWORLD_SCENE_H__
