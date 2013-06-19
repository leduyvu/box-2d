#include "HelloWorldScene.h"
#include "SimpleAudioEngine.h"
using namespace cocos2d;
using namespace CocosDenshion;

#define PTM_RATIO 32
#define DEGTORAD 0.0174532925199432957f

enum {
    kTagParentNode = 1,
};

PhysicsSprite::PhysicsSprite()
: m_pBody(NULL)
{

}

void PhysicsSprite::setPhysicsBody(b2Body * body)
{
    m_pBody = body;
}

bool PhysicsSprite::isDirty(void)
{
    return true;
}

// returns the transform matrix according the Chipmunk Body values
CCAffineTransform PhysicsSprite::nodeToParentTransform(void)
{
    b2Vec2 pos  = m_pBody->GetPosition();

    float x = pos.x * PTM_RATIO;
    float y = pos.y * PTM_RATIO;
    if ( isIgnoreAnchorPointForPosition() ) {
        x += m_obAnchorPointInPoints.x;
        y += m_obAnchorPointInPoints.y;
    }
    // Make matrix
    float radians = m_pBody->GetAngle();
    float c = cosf(radians);
    float s = sinf(radians);
    if( ! m_obAnchorPointInPoints.equals(CCPointZero) ){
        x += c*-m_obAnchorPointInPoints.x + -s*-m_obAnchorPointInPoints.y;
        y += s*-m_obAnchorPointInPoints.x + c*-m_obAnchorPointInPoints.y;
    }
    //Rot, Translate Matrix
    m_sTransform = CCAffineTransformMake( c,  s,
        -s,    c,
        x,    y );
    return m_sTransform;
}

HelloWorld::HelloWorld()
{
    setTouchEnabled( true );
    setAccelerometerEnabled( true );

    CCSize s = CCDirector::sharedDirector()->getWinSize();
    // init physics
    this->initPhysics();

    CCSpriteBatchNode *parent = CCSpriteBatchNode::create("blocks.png", 100);
    m_pSpriteTexture = parent->getTexture();
    addChild(parent, 0, kTagParentNode);
    addNewSpriteAtPosition(ccp(s.width/2, s.height/2));

    CCLabelTTF *label = CCLabelTTF::create("Tap screen", "Marker Felt", 32);
    addChild(label, 0);
    label->setColor(ccc3(0,0,255));
    label->setPosition(ccp( s.width/2, s.height-50));
    CCSprite * r = CCSprite::create("tructhang1.png");
    r->setPosition(ccp(s.width/2,s.height/2));
    //r->setScaleX(10);
    r->setTag(5);
    this->addChild(r);

    rBodyDef.type = b2_dynamicBody;
    rBodyDef.position.Set(s.width/2/PTM_RATIO, s.height/2/PTM_RATIO);
    rBodyDef.userData = r;
    rBodyDef.linearDamping = 0.0f;
    
    rBodyDef.angularDamping = 0.0f;
    body_r = world->CreateBody(&rBodyDef);
    
    // Create shape
    b2PolygonShape rShape;
    rShape.SetAsBox(r->getContentSize().width/2/PTM_RATIO,
                    r->getContentSize().height/PTM_RATIO/2);
    
    // Create shape definition and add to body
    b2FixtureDef rShapeDef;
    rShapeDef.shape = &rShape;
    rShapeDef.density = 0.1f;
    rShapeDef.friction = 0.0f;
    rShapeDef.restitution = 1.0f;
    rShapeDef.filter.groupIndex = -10;
    body_r->CreateFixture(&rShapeDef);
    
    
    CCSprite * r2 = CCSprite::create("paddle.png");
    r2->setTag(8);
    this->addChild(r2);
    myBodyDef.userData = r2;
    myBodyDef.type = b2_dynamicBody;
    myBodyDef.position.Set(15, 11);
    myBodyDef.linearDamping = 100000.0f;
    myBodyDef.angularDamping = 100000.0f;
    body = world->CreateBody(&myBodyDef); 
    
    //kinematicBody->SetLinearVelocity( b2Vec2( 1, 0 ) ); 
    //body->SetAngularVelocity(0.6* 360 * DEGTORAD );
    
    b2PolygonShape rShape2;
    rShape2.SetAsBox(r2->getContentSize().width/2/PTM_RATIO,
                     r2->getContentSize().height/PTM_RATIO/2);
    b2FixtureDef rShapeDef2;
    rShapeDef2.shape = &rShape2;
    rShapeDef2.density = 0.1f;
    rShapeDef2.friction = 0.8f;
    rShapeDef2.restitution = 0.0f;
    rShapeDef2.filter.groupIndex = 10;
    body->CreateFixture(&rShapeDef2);
    
    scheduleUpdate();
}

HelloWorld::~HelloWorld()
{
    delete world;
    world = NULL;
    //delete m_debugDraw;
}

void HelloWorld::initPhysics()
{

    CCSize s = CCDirector::sharedDirector()->getWinSize();

    b2Vec2 gravity;
    gravity.Set(0.0f, -10.0f);
    world = new b2World(gravity);

    // Do we want to let bodies sleep?
    world->SetAllowSleeping(true);

    world->SetContinuousPhysics(true);

    //     m_debugDraw = new GLESDebugDraw( PTM_RATIO );
    //     world->SetDebugDraw(m_debugDraw);

    uint32 flags = 0;
    flags += b2Draw::e_shapeBit;
    //        flags += b2Draw::e_jointBit;
    //        flags += b2Draw::e_aabbBit;
    //        flags += b2Draw::e_pairBit;
    //        flags += b2Draw::e_centerOfMassBit;
    //m_debugDraw->SetFlags(flags);

    // Define the ground body.
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, 0); // bottom-left corner

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    groundBody = world->CreateBody(&groundBodyDef);

    // Define the ground box shape.
    b2EdgeShape groundBox;

    // bottom

    groundBox.Set(b2Vec2(0,0), b2Vec2(s.width/PTM_RATIO,0));
    groundBody->CreateFixture(&groundBox,0);

    // top
    groundBox.Set(b2Vec2(0,s.height/PTM_RATIO), b2Vec2(s.width/PTM_RATIO,s.height/PTM_RATIO));
    groundBody->CreateFixture(&groundBox,0);

    // left
    groundBox.Set(b2Vec2(0,s.height/PTM_RATIO), b2Vec2(0,0));
    groundBody->CreateFixture(&groundBox,0);

    // right
    groundBox.Set(b2Vec2(s.width/PTM_RATIO,s.height/PTM_RATIO), b2Vec2(s.width/PTM_RATIO,0));
    groundBody->CreateFixture(&groundBox,0);
}

void HelloWorld::draw()
{
    // IMPORTANT:
    // This is only for debug purposes
    // It is recommend to disable it
    CCLayer::draw();
    ccGLEnableVertexAttribs( kCCVertexAttribFlag_Position );
    kmGLPushMatrix();
    world->DrawDebugData();
    kmGLPopMatrix();
}

void HelloWorld::addNewSpriteAtPosition(CCPoint p)
{
    CCLOG("Add sprite %0.2f x %02.f",p.x,p.y);
    CCNode* parent = getChildByTag(kTagParentNode);
    //We have a 64x64 sprite sheet with 4 different 32x32 images.  The following code is
    //just randomly picking one of the images
    int idx = (CCRANDOM_0_1() > .5 ? 0:1);
    int idy = (CCRANDOM_0_1() > .5 ? 0:1);
    PhysicsSprite *sprite = new PhysicsSprite();
    sprite->initWithTexture(m_pSpriteTexture, CCRectMake(32 * idx,32 * idy,32,32));
    sprite->autorelease();
    parent->addChild(sprite);
    sprite->setPosition( CCPointMake( p.x, p.y) );
    // Define the dynamic body.
    //Set up a 1m squared box in the physics world
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    body = world->CreateBody(&bodyDef);
    // Define another box shape for our dynamic body.
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(.5f, .5f);//These are mid points for our 1m box

    // Define the dynamic body fixture.
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;    
    fixtureDef.density =2.0f;
    fixtureDef.friction = 0.2f;
    fixtureDef.restitution = 0.5f;
    body->CreateFixture(&fixtureDef);

    sprite->setPhysicsBody(body);
}


void HelloWorld::update(float dt)
{
    //It is recommended that a fixed time step is used with Box2D for stability
    //of the simulation, however, we are using a variable time step here.
    //You need to make an informed choice, the following URL is useful
    //http://gafferongames.com/game-physics/fix-your-timestep/
    
    int velocityIterations = 8;
    int positionIterations = 1;

    // Instruct the world to perform a single step of simulation. It is
    // generally best to keep the time step and iterations fixed.
    world->Step(dt, velocityIterations, positionIterations);
    
    //Iterate over the bodies in the physics world
    for (b2Body* b = world->GetBodyList(); b; b = b->GetNext())
    {
        if (b->GetUserData() != NULL) {
            //Synchronize the AtlasSprites position and rotation with the corresponding body
            CCSprite* myActor = (CCSprite*)b->GetUserData();
            if(myActor->getTag()==8)
            {
                myActor->setPosition( CCPointMake( b->GetPosition().x * PTM_RATIO, b->GetPosition().y * PTM_RATIO) );
            }
            else{
            myActor->setPosition( CCPointMake( b->GetPosition().x * PTM_RATIO, b->GetPosition().y * PTM_RATIO) );
            myActor->setRotation( -1 * CC_RADIANS_TO_DEGREES(b->GetAngle()) );
            }
        }
    }
    rr();
}

void HelloWorld::ccTouchesBegan(cocos2d::CCSet *touches, cocos2d::CCEvent *event)
{
    CCSetIterator it;
    CCTouch* touch;
    for( it = touches->begin(); it != touches->end(); it++)// get all touch point
    {
        touch = (CCTouch*)(*it);
        if(!touch)
            continue;
        CCPoint location = touch->getLocationInView();
        location = CCDirector::sharedDirector()->convertToGL(location);
        b2Vec2 locationWorld = b2Vec2(location.x/PTM_RATIO, location.y/PTM_RATIO);
        {
            b2MouseJointDef md;
            md.bodyA = groundBody;
            md.bodyB = body;
            md.target = locationWorld;
            md.collideConnected = true;
            md.maxForce = 10000.0f * body->GetMass();
            md.dampingRatio = 0;
            md.frequencyHz =100000;
            mouseJoint = (b2MouseJoint *)world->CreateJoint(&md);
            body->SetAwake(true);
        }
    }
}

void HelloWorld::ccTouchesMoved(cocos2d::CCSet *touches, cocos2d::CCEvent *event)
{
    CCSetIterator it;
    CCTouch* touch;
    for( it = touches->begin(); it != touches->end(); it++)// get all touch point
    {
        touch = (CCTouch*)(*it);
        if(!touch)
            continue;
        CCPoint location = touch->getLocationInView();
        location = CCDirector::sharedDirector()->convertToGL(location);
        
        b2Vec2 locationWorld = b2Vec2(location.x/PTM_RATIO, location.y/PTM_RATIO);
        mouseJoint->SetTarget(locationWorld);
    }
    
}

void HelloWorld::ccTouchesEnded(cocos2d::CCSet* touches, cocos2d::CCEvent* event)
{
    mouseJoint->GetBodyA()->GetWorld()->DestroyJoint(mouseJoint);
}

void HelloWorld::prismatic()
{
}

void HelloWorld::createSprite(CCPoint location)
{
    CCSprite * r = CCSprite::create("tructhang1.png");
    r->setTag(8);
    this->addChild(r);
    myBodyDef.userData = r;
    myBodyDef.type = b2_kinematicBody;     //this will be a kinematic body
    myBodyDef.position.Set(location.x/PTM_RATIO,location.y/PTM_RATIO);     // start from left side
    body = world->CreateBody(&myBodyDef);
    
    //kinematicBody->SetLinearVelocity( b2Vec2( 1, 0 ) );         //move right 1 unit per second
    
    //1 turn per second counter-clockwise
    body->SetAngularVelocity(0.6* 360 * DEGTORAD );
    
    b2PolygonShape rShape2;
    rShape2.SetAsBox(r->getContentSize().width/2/PTM_RATIO,
                     r->getContentSize().height/PTM_RATIO/2);
    
    // Create shape definition and add to body
    b2FixtureDef rShapeDef2;
    rShapeDef2.shape = &rShape2;
    rShapeDef2.density = 0.1f;
    rShapeDef2.friction = 0.0f;
    rShapeDef2.restitution = 1.0f;
    
    rShapeDef2.filter.groupIndex = 10;
    body->CreateFixture(&rShapeDef2);
}

void HelloWorld::rr()
{
    
    b2Vec2 anchor1 = body->GetWorldCenter();
    
    b2Vec2 anchor2 = body_r->GetWorldCenter();
    
    b2Vec2 groundAnchor1(body->GetPosition().x * PTM_RATIO, body->GetPosition().y * PTM_RATIO + 1.0f);
    
    b2Vec2 groundAnchor2(body_r->GetPosition().x * PTM_RATIO, body_r->GetPosition().y * PTM_RATIO + 1.0f);
    
    float32 ratio = 1.0f;
    b2PulleyJointDef jointDef;
    jointDef.Initialize(body, body_r, groundAnchor1, groundAnchor2, anchor1, anchor2, ratio);
    world->CreateJoint(&jointDef);
    
    //body->set
    

    //float32 GetLengthA(30);
    //float32 GetLengthB(30);
    //body->SetAwake(true);
    //jointDef.
}

CCScene* HelloWorld::scene()
{
    // 'scene' is an autorelease object
    CCScene *scene = CCScene::create();
    
    // add layer as a child to scene
    CCLayer* layer = new HelloWorld();
    scene->addChild(layer);
    layer->release();
    return scene;
}