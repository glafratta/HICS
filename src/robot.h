#ifndef ROBOT_H
#define ROBOT_H
//box2d robot body and kinematic model
#include "box2d/box2d.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
const float SAFE_ANGLE =M_PI_2;
const float MAX_TURN =M_PI;
const float ROBOT_HALFLENGTH =0.135; //uncomment for robot
const float ROBOT_HALFWIDTH =0.09; //real
const float ROBOT_BOX_OFFSET_X= -0.09;
const float ROBOT_BOX_OFFSET_Y =0;
const float ROBOT_BOX_OFFSET_ANGLE =0;
const float BETWEEN_WHEELS =.15;
const float MAX_SPEED=.2;//.196; //1.56
const float MAX_OMEGA =2; //2 radians
const float ANGLE_ERROR_TOLERANCE =5 * M_PI/180;
const float BOX2DRANGE =1.0;
const float LIDAR_RANGE =1.1;
const float HZ =40.0;
const float MAX_ANGLE_ERROR= M_PI;
const float SAFE_DISTANCE =sqrt(ROBOT_HALFLENGTH*ROBOT_HALFLENGTH+ROBOT_HALFWIDTH*ROBOT_HALFWIDTH); //DEFAULT backtracking distance
const float MAX_DISTANCE_ERROR =2*BOX2DRANGE;
const float MOTOR_CALLBACK =.1; //MOTOR CALL BACK EVERY .1 s
const float LIDAR_SAMPLING_RATE=0.2;
const float TURN_FRICTION =.9; //.75
const float DISTANCE_ERROR_TOLERANCE=.02;
const int maxNodesOnSpot =4;
const float TRACKING_ERROR_TOLERANCE = MAX_SPEED*MOTOR_CALLBACK*0.5; //OG *0.5
const float TRACKING_ANGLE_TOLERANCE =MAX_OMEGA*0.5*MOTOR_CALLBACK;
const float FORGET_THRESHOLD=0.05;
//camera filtering
const int POS_IT=2;
const int VEL_IT=6;
const int FPS=30;
const int order=3;
const int DC=0; //HZ
const int cutoff_frequency=4; //HZ
const int band_width=0.5;

class Robot {
private: 
	b2FixtureDef fixtureDef;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		body->GetUserData().pointer = reinterpret_cast<uintptr_t>(this);
		b2Vec2 center(ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
		b2PolygonShape box;
		box.SetAsBox(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH, center, ROBOT_BOX_OFFSET_ANGLE);
		fixtureDef.shape = &box;
		fixtureDef.friction =0;
		body->CreateFixture(&fixtureDef);
		
	}
};


#endif

