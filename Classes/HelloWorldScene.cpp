#include "HelloWorldScene.h"
#include "utils.h"
#include "Vehicle.h"
#include "SteeringBehavior.h"

USING_NS_CC;

Scene* HelloWorld::createScene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();
    
    // 'layer' is an autorelease object
    auto layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }
    
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

	m_cxClient = visibleSize.width;
	m_cyClient = visibleSize.height;
	m_bPaused = false;
	m_vCrosshair = Point(cxClient()/2.0,cyClient()/2.0);
	Point spawnPos = Point(CCRANDOM_0_1()*m_cxClient , CCRANDOM_0_1()*m_cyClient);

	m_drawNode = DrawNode::create();
	this->addChild(m_drawNode);

	//this->CreateObstacles();
	//this->CreateWalls();

	//GameWorld
	

	/*Sprite* tempSprite = Sprite::create("1.jpg");
	testVehicle =  Vehicle::createWithInfo(Point::ZERO,CCRANDOM_0_1()*TwoPi,Point(0,0),1.0,2.0,150.0,0,10,tempSprite);
	this->addChild(testVehicle);
	Sprite* tempSprite3 = Sprite::create("1.jpg");
	testVehicle2 =  Vehicle::createWithInfo(Point::ZERO,CCRANDOM_0_1()*TwoPi,Point(0,0),1.0,2.0,150.0,0,10,tempSprite3);
	this->addChild(testVehicle2);
	randMoveTo();*/
	//Point spawnPos = m_vCrosshair;
	Sprite* tempSprite2 = Sprite::create("1.jpg");
	Vehicle* pVehicle = Vehicle::createWithInfo(Point::ZERO,CCRANDOM_0_1()*TwoPi,Point(0,0),1.0,1000,140,2*TwoPi,10,tempSprite2);
	pVehicle->scheduleUpdate();
	this->addChild(pVehicle);
	m_Vehicles.pushBack(pVehicle);

	/*pVehicle->Steering()->obstacleAvoidanceOn();
	pVehicle->Steering()->wanderOn();*/
	//pVehicle->Steering()->arriveOn();
	pVehicle->Steering()->followPathOn();

	Path* path = pVehicle->Steering()->m_pPath;
	std::list<cocos2d::Point> wayPoints = path->GetPath();
	for (std::list<cocos2d::Point>::iterator it = wayPoints.begin(); it != wayPoints.end(); it++)
	{
		Point point = *it;
		m_drawNode->drawDot(point,5,ccc4f(0.5f, 0.5f, 1, 1));
	}

	//Sprite* tempSprite3 = Sprite::create("1.jpg");
	//Vehicle* pVehicle2 = Vehicle::createWithInfo(m_vCrosshair,CCRANDOM_0_1()*TwoPi,Point(0,0),1.0,50,150,0,10,tempSprite3);
	//pVehicle2->scheduleUpdate();
	//this->addChild(pVehicle2);
	//m_Vehicles.pushBack(pVehicle2);

	//pVehicle2->Steering()->obstacleAvoidanceOn();
	////pVehicle2->Steering()->wanderOn();
	//pVehicle2->Steering()->pHunter = pVehicle;
	//pVehicle2->Steering()->hideOn();
	
	

    return true;
}

void HelloWorld::randMoveTo()
{
	Point target = Point(CCRANDOM_0_1()*cxClient() , CCRANDOM_0_1()*cyClient());
	//Sequence* seq = Sequence::createWithTwoActions(MoveTo::create(5,target),CallFunc::create(this,callfunc_selector(HelloWorld::randMoveTo)));
	testVehicle->runAction(MoveTo::create(5,target));

	Point target2 = Point(CCRANDOM_0_1()*cxClient() , CCRANDOM_0_1()*cyClient());
	Sequence* seq2 = Sequence::createWithTwoActions(MoveTo::create(5,target2),CallFunc::create(this,callfunc_selector(HelloWorld::randMoveTo)));
	testVehicle2->runAction(seq2);
	
}

void HelloWorld::CreateObstacles()
{
	
	Point pos1 = Point(m_cxClient*0.25,m_cyClient*0.5);
	Point pos2 = Point(m_cxClient*0.5,m_cyClient*0.5);
	Point pos3 = Point(m_cxClient*0.75,m_cyClient*0.5);
	Point pos4 = Point(m_cxClient*0.5,m_cyClient*0.75);
	Point pos5 = Point(m_cxClient*0.5,m_cyClient*0.25);

	float radius1 = 60;
	float radius2 = 60;
	float radius3 = 60;
	float radius4 = 60;
	float radius5 = 60;

	m_drawNode->drawDot(pos1,radius1,ccc4f(0.5f, 0.5f, 1, 1));
	m_drawNode->drawDot(pos2,radius2,ccc4f(0.5f, 0.5f, 1, 1));
	m_drawNode->drawDot(pos3,radius3,ccc4f(0.5f, 0.5f, 1, 1));
	m_drawNode->drawDot(pos4,radius4,ccc4f(0.5f, 0.5f, 1, 1));
	m_drawNode->drawDot(pos5,radius5,ccc4f(0.5f, 0.5f, 1, 1));

	Obstacle* obs1 = new Obstacle();
	obs1->position = pos1;
	obs1->radius = radius1;
	Obstacle* obs2 = new Obstacle();;
	obs2->position = pos2;
	obs2->radius = radius2;
	Obstacle* obs3 = new Obstacle();;
	obs3->position = pos3;
	obs3->radius = radius3;
	Obstacle* obs4 = new Obstacle();;
	obs4->position = pos4;
	obs4->radius = radius4;
	Obstacle* obs5 = new Obstacle();;
	obs5->position = pos5;
	obs5->radius = radius5;

	m_Obstacles.push_back(obs1);
	m_Obstacles.push_back(obs2);
	m_Obstacles.push_back(obs3);
	m_Obstacles.push_back(obs4);
	m_Obstacles.push_back(obs5);
}

void HelloWorld::CreateWalls()
{
	Point startPos = Point(0,m_cyClient*0.05);
	Point endPos = Point(m_cxClient,m_cyClient*0.05);
	Point temp = ccpNormalize(endPos - startPos);
	Point forceDirect = Point(-temp.y,temp.x);
	m_drawNode->drawSegment(startPos,endPos,5,ccc4f(0.5f, 0.5f, 1, 1));
	Wall2D wall;
	wall.startPos = startPos;
	wall.endPos = endPos;
	wall.forceDirect = forceDirect;
	m_Walls.push_back(wall);

	Point startPos2 = Point(m_cxClient,m_cyClient*0.95);
	Point endPos2 = Point(0,m_cyClient*0.95);
	Point temp2 = ccpNormalize(endPos2 - startPos2);
	Point forceDirect2 = Point(-temp2.y,temp2.x);
	m_drawNode->drawSegment(startPos2,endPos2,5,ccc4f(0.5f, 0.5f, 1, 1));
	Wall2D wall2;
	wall2.startPos = startPos2;
	wall2.endPos = endPos2;
	wall2.forceDirect = forceDirect2;
	m_Walls.push_back(wall2);
}
