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

    /////////////////////////////
    // 2. add a menu item with "X" image, which is clicked to quit the program
    //    you may modify it.

    // add a "close" icon to exit the progress. it's an autorelease object
    auto closeItem = MenuItemImage::create(
                                           "CloseNormal.png",
                                           "CloseSelected.png",
                                           CC_CALLBACK_1(HelloWorld::menuCloseCallback, this));
    
	closeItem->setPosition(Vec2(origin.x + visibleSize.width - closeItem->getContentSize().width/2 ,
                                origin.y + closeItem->getContentSize().height/2));

    // create menu, it's an autorelease object
    auto menu = Menu::create(closeItem, NULL);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);

    /////////////////////////////
    // 3. add your codes below...

    // add a label shows "Hello World"
    // create and initialize a label
    
    auto label = LabelTTF::create("Hello World", "Arial", 24);
    
    // position the label on the center of the screen
    label->setPosition(Vec2(origin.x + visibleSize.width/2,
                            origin.y + visibleSize.height - label->getContentSize().height));

    // add the label as a child to this layer
    this->addChild(label, 1);

    // add "HelloWorld" splash screen"
    auto sprite = Sprite::create("HelloWorld.png");

    // position the sprite on the center of the screen
    sprite->setPosition(Vec2(visibleSize.width/2 + origin.x, visibleSize.height/2 + origin.y));

    // add the sprite as a child to this layer
    //this->addChild(sprite, 0);

	//GameWorld
	
	//m_cxClient(cx),
	//	m_cyClient(cy),
	//	m_bPaused(false),
	//	m_vCrosshair(Vector2D(cxClient()/2.0, cxClient()/2.0)),
	//	//determine a random starting position
	//	Vector2D SpawnPos = Vector2D(cx/2.0+RandomClamped()*cx/2.0,
	//	cy/2.0+RandomClamped()*cy/2.0);


	//Vehicle* pVehicle = new Vehicle(this,
	//	SpawnPos,                 //initial position
	//	RandFloat()*TwoPi,        //start rotation
	//	Vector2D(0,0),            //velocity
	//	Prm.VehicleMass,          //mass
	//	Prm.MaxSteeringForce,     //max force
	//	Prm.MaxSpeed,             //max velocity
	//	Prm.MaxTurnRatePerSecond, //max turn rate
	//	Prm.VehicleScale);        //scale

	//pVehicle->Steering()->FlockingOn();

	//m_Vehicles.push_back(pVehicle);
	m_cxClient = visibleSize.width;
	m_cyClient = visibleSize.height;
	m_bPaused = false;
	m_vCrosshair = Point(cxClient()/2.0,cyClient()/2.0);
	//Point spawnPos = Point(CCRANDOM_0_1()*m_cxClient , CCRANDOM_0_1()*m_cyClient);
	Point spawnPos = m_vCrosshair;
	Sprite* tempSprite = Sprite::create("1.jpg");
	Vehicle* pVehicle = Vehicle::createWithInfo(spawnPos,CCRANDOM_0_1()*TwoPi,Point(0,0),1.0,2.0,150.0,360.0,10,tempSprite);
	pVehicle->scheduleUpdate();
	this->addChild(pVehicle);
	pVehicle->Steering()->Seek(Point(cxClient(),cyClient()));
	m_Vehicles.pushBack(pVehicle);
	
    return true;
}


void HelloWorld::menuCloseCallback(Ref* pSender)
{
#if (CC_TARGET_PLATFORM == CC_PLATFORM_WP8) || (CC_TARGET_PLATFORM == CC_PLATFORM_WINRT)
	MessageBox("You pressed the close button. Windows Store Apps do not implement a close button.","Alert");
    return;
#endif

    Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
    exit(0);
#endif
}
