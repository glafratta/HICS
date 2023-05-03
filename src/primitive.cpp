#include "primitive.h"
#include "robot.h"



Primitive::simResult Primitive::willCollide(b2World & _world, int iteration, bool debugOn=0, b2Vec2 start = b2Vec2(), float _theta=0.0, float remaining=8.0){ //CLOSED LOOP CONTROL, og return simreult
		simResult result = simResult(simResult::resultType::successful);
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		FILE * robotPath;
		if (debugOn){
			sprintf(planFile, "/tmp/robot%04i.txt", iteration);
			robotPath = fopen(planFile, "a");
		}
		float theta = _theta;
		b2Vec2 instVelocity = {0,0};
		robot.body->SetTransform(start, theta);
		int step=0;
		for (step; step < (hz*remaining); step++) {//3 second
			instVelocity.x = RecordedVelocity.Length()*cos(theta); //integrate?
			instVelocity.y = RecordedVelocity.Length()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions
			}
			_world.Step(1.0f/hz, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getOmega()/hz; //= omega *t
			if (obstacle.isValid()){
				float absAngleToObstacle = abs(obstacle.getAngle(robot.body));

				if (absAngleToObstacle>=endAvoid){
					break;
				}
			}
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				if (type == Primitive::Type::BASELINE && step/hz >REACTION_TIME){ //stop 2 seconds before colliding so to allow the robot to explore
						b2Vec2 posReadjusted;
						posReadjusted.x = start.x+ instVelocity.x*(step/hz-REACTION_TIME);						
						posReadjusted.y = start.y+ instVelocity.y*(step/hz-REACTION_TIME);						
						robot.body->SetTransform(posReadjusted, _theta); //if the simulation crashes reset position for 
						result = simResult(simResult::resultType::safeForNow, Object(ObjectType::obstacle, listener.collisions[index]));
				}
				else{
						result = simResult(simResult::resultType::crashed, Object(ObjectType::obstacle, listener.collisions[index]));
						robot.body->SetTransform(start, _theta); //if the simulation crashes reset position for 
						result.collision.safeForNow =0;

					}
				break;
			}
		}	
		b2Vec2 distance; //= robot.body->GetPosition();
		distance.x = robot.body->GetPosition().x - start.x;
		distance.y = robot.body->GetPosition().y - start.y;
		result.distanceCovered = distance.Length() ;
		result.endPose = robot.body->GetTransform();
		int roboCount=0;
		for (b2Body * b = _world.GetBodyList(); b!=NULL; b = b->GetNext()){
			if (b->GetUserData()!=NULL){
				roboCount++;
			}
			_world.DestroyBody(b);

		}
		if (debugOn){
			fclose(robotPath);
		}

		return result;
	
}


void Primitive::trackObject(Object & object, float timeElapsed, b2Vec2 robVelocity, b2Vec2 robPos){ //isInternal refers to whether the tracking is with respect to the global coordinate frame (i.e. in willCollide) if =1, if isIntenal =0 it means that the object is tracked with the robot in the default position (0.0)
	b2Vec2 shift = {-robVelocity.x*timeElapsed, -robVelocity.y*timeElapsed}; //calculates shift in the time step
	b2Vec2 newPos(object.getPosition().x+shift.x,object.getPosition().y + shift.y);
	object.setPosition(newPos);
	float angle = object.getAngle(robVelocity);
	object.setAngle(angle); //with respect to robot's velocity
}

Primitive::controlResult Primitive::controller(){
float recordedAngle = atan(RecordedVelocity.y/RecordedVelocity.x);
float tolerance = 0.01; //tolerance in radians/pi = just under 2 degrees degrees
    if (obstacle.isValid()){
        float obstacleAngle = atan(obstacle.getPosition().y/obstacle.getPosition().x);
        float angleDifference = obstacleAngle - recordedAngle;
        if (abs(angleDifference) >= endAvoid){
			obstacle.invalidate();
            return DONE;
        }
    }
    else {
		float timeStepError =action.getOmega()*0.2 - recordedAngle; 
        accumulatedError += timeStepError; //og was new variable angleerror
		if (timeStepError<tolerance){
			action.LeftWheelSpeed = 0.5;
			action.RightWheelSpeed = 0.5;
		}
		else{
			float normAccErr = timeStepError/M_PI_2;
				action.LeftWheelSpeed -= normAccErr*pGain;  //og angle was -angle
				action.RightWheelSpeed += normAccErr *pGain; //og was + angle
				if (action.LeftWheelSpeed>1.0){
				action.LeftWheelSpeed=1.0;
				}
				if (action.RightWheelSpeed>1.0){
					action.RightWheelSpeed=1;
				}
				if (action.LeftWheelSpeed<(-1.0)){
					action.LeftWheelSpeed=-1;
				}
				if (action.RightWheelSpeed<(-1.0)){
					action.RightWheelSpeed=-1;
				}


		}
    }
    return CONTINUE;
}
