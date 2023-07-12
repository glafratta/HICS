#ifndef ROBOT_H
#define ROBOT_H
//box2d robot body and kinematic model
#include "Box2D/Box2D.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define SAFE_ANGLE M_PI_2
#define SAFE_DISTANCE 0.125
#define MAX_TURN M_PI
#define ROBOT_HALFLENGTH 0.11
#define ROBOT_HALFWIDTH 0.09
#define ROBOT_BOX_OFFSET_X -0.05
#define ROBOT_BOX_OFFSET_Y 0
#define ROBOT_BOX_OFFSET_ANGLE 0
#define BETWEEN_WHEELS .15
#define MAX_SPEED 0.125
#define MAX_OMEGA M_PI
#define ANGLE_ERROR_TOLERANCE 5 * M_PI/180;
#define BOX2DRANGE 1.0
#define LIDAR_RANGE 1.0
#define REACTION_TIME 1.0
#define HZ 50.0
#define MAX_ANGLE_ERROR M_PI
#define MAX_DISTANCE_ERROR BOX2DRANGE

const int maxNodesOnSpot = 2*MAX_TURN/(M_PI_2-atan(ROBOT_HALFWIDTH/ROBOT_HALFLENGTH));
										//this is the angle that the robot would have to turn
										//to be at a safe distance from an obstacle in the worst
										//case scenario, provided that the position of the obstacle
										//refers to the centre of mass of the robot


class Robot {
private: 
	b2FixtureDef fixtureDef;
	//b2PolygonShape box;
	//b2Vec2 centroid;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);  
		b2Vec2 center(ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
		b2PolygonShape box;
		box.SetAsBox(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH, center, ROBOT_BOX_OFFSET_ANGLE);
		//centroid = box.m_centroid; 
		fixtureDef.shape = &box;
		fixtureDef.friction =0;
		body->CreateFixture(&fixtureDef);
		
	}
};


#endif

