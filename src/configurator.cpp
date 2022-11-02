#pragma once
#include "configurator.h"
#include <chrono>
#include <iostream>

void Configurator::NewScan(){ 
	std::vector <cv::Point2f> current;
	iteration++; //iteration set in getVelocity
				////////FOR DEBUGGING ////////////////////////
		char filePath[256];
		sprintf(filePath, "%s/%s%04i.dat", folder, readMap,iteration);
        printf("%s\t", filePath);
		std::ifstream file(filePath);

        float x, y;
		//box2d->previous = box2d->current;
		//box2d->current.clear();
		while (file>>x>>y){
			current.push_back(cv::Point2f(x, y));

			
		}
		file.close();

		//END PASTED SECTIOM

		//printf("begin newScan plan size %i\n", plan.size());
		bool isObstacleStillThere=0;
		//see time delay between scans to calculate speed more accurately

        auto now =std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
		timeElapsed=float(diff.count())/1000; //express in seconds
		printf("time elapsed between newscans = %f ms\n", timeElapsed);
		totalTime += timeElapsed; //for debugging
		if ( timeElapsed< samplingRate){
			timeElapsed= samplingRate;
		}
		//printf("time elapsed = %f ms\n", timeElapsed);
		previousTimeScan=now; //update the time of sampling
		//WRITE TO FILE (debug) + WORLD BUILDING
		b2World world = b2World({0.0f,0.0f});
		char name[256];
		// sprintf(name, "/tmp/bodies%04d.txt", iteration);
		// FILE *file= fopen(name, "w+");
		// printf("%s\n", name);
	

//		printf("current size: %i\n", current.size());

		
		//put velocity calculations before world creation so that we can check if obstacle is still there
		
		Configurator::getVelocityResult affineTransResult= GetRealVelocity(current);
		b2Vec2 estimatedVelocity;

		//printf("estimatedVelocity = %f, %f\n", estimatedVelocity.x, estimatedVelocity.y);
		if (affineTransResult.valid){
			estimatedVelocity.x = affineTransResult.vector.x; //m/s
			estimatedVelocity.y = affineTransResult.vector.y; //m/s
		}
		else{
			estimatedVelocity = {0.0f, 0.0f};
			//return 2; //if returns 2 it means its time to panic
		}
		absPosition.x += estimatedVelocity.x*timeElapsed;
		absPosition.y += estimatedVelocity.y*timeElapsed;
		dumpPath = fopen("/tmp/dumpPath.txt", "a+");
		fprintf(dumpPath, "%f\t%f\n", absPosition.x, absPosition.y);
		fclose(dumpPath);
		//if obstacles are out of sight, clear them from memory (can this be done with a filter)
		if (plan.size()>0 && plan[0].obstacle.isValid()){


			plan[0].trackObject(plan[0].obstacle, timeElapsed, estimatedVelocity, {0.0f, 0.0f}); //robot default position is 0,0
			if (plan[0].obstacle.getAngle() >= M_PI_2){ 		//find angle between robot body and collision 		//if angle larger than 90 deg delete this state in the plan and recheck
				plan.erase(plan.begin());
				printf("erased plan\n");
			}
			else{
				State::Object temp = plan[0].obstacle;
				plan[0]= State(temp);
			}

		}






		//printf("before making body\n");
		for (int i=0; i<current.size(); i++){ //makes obstacles and checks for duplicates
			//b2Vec2 pointCurrent(roundf((current[i].x)*100.f)/100.f, roundf((current[i].y)*100.f)/100.f);
			//b2Vec2 pointPrevious(roundf((current[i-1].x)*100.f)/100.f, roundf((current[i-1].y)*100.f)/100.f);

			if ((current[i].x != current[i-1].x || current[i].y!=current[i-1].y)&& current[i].y>=0 && sqrt(pow(current[i].x,2)+pow(current[i].y,2))<1){ // robot only sees obstacles ahead of it
				//printf("making body!\n");
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (current[i] == cv::Point2f(0.0f, 0.0f)){
					printf("obstacle at 0,0, not normal\n");
					printf("body n. %i\n", i);
				}
				//can get transform from body
			//printf("plan size %i\n", plan.size());
				if (plan.size()>0 && plan[0].obstacle.isValid()){
					//printf("plan is there and obstacle valid\n");
					// printf("point %f, %f\n", pointCurrent.x, pointCurrent.y);
					// printf("pointCurrent.x > plan[0].obstacle.getPosition().x-0.05 : %i\n ", pointCurrent.x > plan[0].obstacle.getPosition().x-0.05);
					// printf("pointCurrent.x < plan[0].obstacle.getPosition().x+0.05 : %i\n ", pointCurrent.x < plan[0].obstacle.getPosition().x+0.05);
					// printf("pointCurrent.y > plan[0].obstacle.getPosition().y-0.05 : %i\n ", pointCurrent.y > plan[0].obstacle.getPosition().y-0.05);
					// printf("pointCurrent.y < plan[0].obstacle.getPosition().y+0.05 : %i\n ", pointCurrent.y < plan[0].obstacle.getPosition().y+0.05);
					//printf("all together: %i", (((plan[0].obstacle.getPosition().x)-0.05<pointCurrent.x<(plan[0].obstacle.getPosition().x)+0.05) and ((plan[0].obstacle.getPosition().y)-0.05<pointCurrent.y<(plan[0].obstacle.getPosition().y)+0.05)));

					if (current[i].x > plan[0].obstacle.getPosition().x-0.05 && current[i].x < plan[0].obstacle.getPosition().x+0.05 && current[i].y > plan[0].obstacle.getPosition().y-0.05 && current[i].y < plan[0].obstacle.getPosition().y+0.05){
						isObstacleStillThere =1;
						//printf("changed still there\n");
					}
				}
					bodyDef.position.Set(current[i].x, current[i].y); 
					body = world.CreateBody(&bodyDef);
					body->CreateFixture(&fixtureDef);
					//fprintf(file, "%f\t%f\n", body->GetPosition().x, body->GetPosition().y);

			}
		}
		printf("body count = %i\n", world.GetBodyCount());
		//fclose(file);
		//printf("obstacle is still there: %i\n", isObstacleStillThere);
		//printf("bodies created\n");
		if (!isObstacleStillThere && plan.size()>0){
			plan.erase(plan.begin());
		}

		current.clear();

		//once the representation is updated see if you're going to crash

		//DONE	//willCollide should follow a trajectory determined by the position of the obstacle


		//if no obstacles to keep track of, check if with the current trajectory it will bump into something else
		//if the plan vector is empty the state is the default state
		//printf("plan size %i\n", plan.size());
		if (plan.size() ==0){
			printf("state is desired state\n");
		desiredState.setRecordedVelocity(estimatedVelocity); //logs in previous recorded velocity in case affine transformation fails
		//TO DO		//generate error from estimatedVelocity
		State::simResult result =desiredState.willCollide(world, iteration);
			if (result.resultCode == State::simResult::crashed){
				State newState(result.collision);
				//generate new trajectory for this state: AUTOMATIC in state constructor
				plan.push_back(newState); //if state has one or more obstacles it is "avoid" state
				//TO DO: //start computing a plan to avoid obstacle (trajectory) -just obstacle
				printf("collided, new state has trajectory omega= %f,  linear speed: %f\n", newState.trajectory.getOmega(), newState.trajectory.getLinearSpeed() );
			}
		}
		else if (plan.size()>0){ //the program enters this loop both if a disturbance has just been detected and if it was previously detected
			plan[0].setRecordedVelocity(estimatedVelocity);
			 //check if the trajectory being followed will bump the robot into something
			State::simResult result =plan[0].willCollide(world, iteration);
			if (result.resultCode == State::simResult::crashed){
				State newState(result.collision);
				//generate new trajectory for this state: AUTOMATIC in state constructor
				plan.push_back(newState); //if state has one or more obstacles it is "avoid" state
				//TO DO: //start computing a plan to avoid obstacle (trajectory) -just obstacle
				printf("plan[0] collided, new state has trajectory omega= %f, linear speed: %f\n", newState.trajectory.getOmega(), newState.trajectory.getLinearSpeed() );
			}

		}

		
		//compare detected velocity with predicted velocity and correct motor output to minimize error


	}

Configurator::getVelocityResult Configurator::GetRealVelocity(std::vector <cv::Point2f> current){	 //does not modify current vector, creates copy	
		getVelocityResult result;
		MedianFilter filterCurrent;
		MedianFilter filterPrevious;
		//std::vector <cv::Point2f> currentTmp =current;
		std::vector <cv::Point2f> previousTmp;
		FILE * currentSmooth;
		char smoothName[250];
		sprintf(smoothName,"%s/%s%04d_smooth.dat", folder, readMap, iteration);
		currentSmooth=fopen(smoothName, "w");
		//filter current
		for (cv::Point2f p:current){
			filterCurrent.applyToPoint(p);
			fprintf(currentSmooth, "%f\t%f\n", p.x, p.y);
		}
		fclose(currentSmooth);
		if (iteration >1){
			char prevPath[256];
			sprintf(prevPath, "%s/%s%04d_smooth.dat", folder, readMap, iteration-1);
        	printf("getting previous from %s\n", prevPath);
			std::ifstream prev(prevPath);
			float x,y;
			while (prev>>x>>y){
				cv::Point2f p(x,y);
				filterPrevious.applyToPoint(p);
				previousTmp.push_back(cv::Point2f(x,y));
			}
			prev.close();
		}
		//iteration++;

		int diff = current.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger
		//printf("current: %i, previous: %i\n", current.size(), previousTmp.size());


        //adjust for discrepancies in vector size		//int diff = currSize-prevSize;
		if(diff>0){ //(current.size()>previous.size()){
			if (previousTmp.empty()){
				for (cv::Point p:current){
					previousTmp.push_back(cv::Point());
				} }
			else{
				for (int i=0; i<abs(diff); i++){
					previousTmp.push_back(previousTmp[0]); //before it was [-1]
				if (previousTmp[-1].x == 0 && previousTmp[-1].y ==0){
					printf("previous fucked up\n");
				}

			}
			}
		}
	
		else if (diff<0){//(current.size()<previous.size()){
			if (current.empty()){
				for (cv::Point p:previousTmp){
					current.push_back(cv::Point());
					printf("no data");
					return result;
				} }
			else{
				for (int i=0; i<abs(diff); i++){
			current.push_back(current[0]);
				if (current[-1].x == 0 && current[-1].y ==0){
					printf("current fucked up\n");
				}

				}
		}
		//printf("p>curr\n");
		}

	//use partial affine transformation to estimate displacement
	cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, current, cv::noArray(), cv::LMEDS);
	float theta;
			//printf("matrix is empty? %i\n", transformMatrix.empty());
			if (!transformMatrix.empty()){
				b2Vec2 tmp;
				tmp.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
				tmp.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
				if (tmp.Length()>maxAbsSpeed){
					float tmpAngle = atan2(tmp.y, tmp.x);
					printf("tmpAngle = %f pi\n", tmpAngle/M_PI);
					tmp.x = maxAbsSpeed * cos(tmpAngle);
					tmp.y = maxAbsSpeed *sin(tmpAngle);
				}
				return getVelocityResult(tmp);
			}
			else if (plan.empty()){ //if the plan is empty look at the default wheel speed
				State * state;
				if (plan.size()>0){
					state = &plan[0];
				}
				else{
					state = &desiredState;
				}
				b2Vec2 estimatedVel;
				theta = state->getTrajectory().getOmega()* timeElapsed;
				estimatedVel ={state->getTrajectory().getLinearSpeed()*cos(theta),state->getTrajectory().getLinearSpeed()*sin(theta)};
				result = getVelocityResult(estimatedVel);
				printf("getting velocity from current state\n");
				return result;
			}
			// else if (plan.size() >0 && transformMatrix.empty()){ //if an obstacle is being avoided use the set speed
			// b2Vec2 estimatedVel = plan[0].trajectory.getVelocity();
			// 	result = getVelocityResult(estimatedVel);
			// 	printf("getting velocity from plan state\n");
			// 	return result;
			// }
			else{
				printf("could not find velocity\n");
				return result;
			}
		}
	//}



b2Vec2 Configurator::estimateDisplacementFromWheels(){ //actually estimates displacement
	//we 
    b2Vec2 vel;
	float realL = maxAbsSpeed*leftWheelSpeed;
	float realR = maxAbsSpeed*rightWheelSpeed;
    	//find angle theta in the pose:
	float theta = (realL-realR)*timeElapsed/desiredState.getTrajectory().getDistanceWheels(); //rad/s, final angle at end of 1s
	//find absolute speed
	float V =(realL+realR)/2; //distance covered
	//find velocity vector: this is per second so no need to change this; but it is the instantaneous velocity
	//vel= b2Vec2(speed*sin(angVel), speed*(-cos(angVel)));
    //from the derivation (see notes on robot kinematics)
	//////below is wrong but that's how we find pose
	// vel.x = ((V/(theta)) * sin(theta*timeElapsed))*timeElapsed; //i added theta *timeElapsed because idkz
	// vel.y = ((V/theta)*(1-cos(theta*timeElapsed)))*timeElapsed;
	// /////////////////CORRECT VELOCITY: we just take the derivative of the formula used to find pose, which is in the notes
	// vel.x = ((realL+ realR)*cos(theta)/2); //by multiplying by timeElapsed we get how far we expect the robot to get in the next scan
	// vel.y = ((realL+ realR)*sin(theta)/2);
	//printf("r = %f\t theta = %f pi\n", V, theta/M_PI);
	if (realR-realL == 0){
		vel.x = 0;
		vel.y = realL;
		}
	else {
		vel.x = (desiredState.getTrajectory().getDistanceWheels()/2)* sin(desiredState.trajectory.getDistanceWheels()*timeElapsed/(realR-realL));
		vel.y = -(desiredState.getTrajectory().getDistanceWheels()/2)* cos(desiredState.getTrajectory().getDistanceWheels()*timeElapsed/(realR-realL));

	}
	return vel;
}

void Configurator::controller(){
//FIND ERROR
b2Vec2 desiredPosition, nextPosition, recordedPosition; //target for genearting a corrective trajectory
State * state;
if (plan.size()==0){
    state = &desiredState;
}
else if (plan.size()>0){
    //state = &plan[0];
	leftWheelSpeed = plan[0].getTrajectory().getLWheelSpeed();
	rightWheelSpeed = plan[0].getTrajectory().getRWheelSpeed();
	return;
}
double angleError=0;
double distanceError=0;

printf("iteration in controller: %i\n", iteration);

if (iteration > 0){
    float x,y, t;
    t=0; //discrete time
    char name[50];
    sprintf(name, "%s", fileNameBuffer); //
    //printf("%s\n", name);
    std::ifstream file(name);

    while (file>>x>>y){ 
        t= t+ 1.0f/60.0f;
        if(totalTime<t && t<=(totalTime+1/60.f)){ 
            desiredPosition = b2Vec2(x,y);
            printf("desired position out of file: %f, %f\t time recorded as: %f, timeElapsed: %f\n", x, y,t, timeElapsed);
            break;
        }
        // else {
        //     leftWheelSpeed = 0;
        //     rightWheelSpeed = 0;
        // }

    }

    file.close();
    //desiredVelocity =b2Vec2(desiredPosition.x/timeElapsed, desiredPosition.y/timeElapsed);
    recordedPosition = {state->getRecordedVelocity().x, state->getRecordedVelocity().y};
    printf("recordedpos = %f, %f\n", recordedPosition.x, recordedPosition.y);
    //float desiredAngle = atan2(recordedPosition.y)
   // angleError = atan2(desiredPosition.x, desiredPosition.y)-atan2(recordedPosition.x, recordedPosition.y); //flag
    angleError = atan2(desiredPosition.y, desiredPosition.x)-atan2(recordedPosition.y, recordedPosition.x); //flag    
    //normalise error
    double maxError = 2* M_PI;
    angleError /= maxError;


    printf("desired angle: %f pi\t recorded angle: %f pi\n", atan2(desiredPosition.y, desiredPosition.x)/M_PI, atan2(recordedPosition.y, recordedPosition.x)/M_PI);
    //printf("desired position = %f, %f\trecorded position: %f, %f\n", desiredPosition.x, desiredPosition.y, recordedPosition.x, recordedPosition.y);
    printf("angleError =%f\n", angleError);
    distanceError = desiredPosition.Length() - recordedPosition.Length();
    printf("distanceError = %f\n", distanceError);
    }

//    // if (angleError>0){
//         leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain; //if angle is larger than 0 means thta the robot is going to R, slow L wheel
//         printf("initial L wheel speed: %f\n", state->getTrajectory().getLWheelSpeed());
//     //}
//     //else if (angleError<0){
//         rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain;  //viceversa, sign is +ve because the angle error is neg
//         printf("initial R wheel speed: %f\n", state->getTrajectory().getRWheelSpeed());
//             //}

//negative angle error means too much to the R: so slow down L wheel and speed up right
//positive angle means too much to the left, slow down R and speed up left

leftWheelSpeed = state->getTrajectory().getLWheelSpeed() + angleError*gain+ distanceError*gain;  //og angle was +angle
rightWheelSpeed = state->getTrajectory().getRWheelSpeed()- angleError *gain + distanceError*gain; //og was - angle

if (leftWheelSpeed>1.0){
    leftWheelSpeed=1.0;
}
if (rightWheelSpeed>1.0){
    rightWheelSpeed=1;
}
if (leftWheelSpeed<(-1.0)){
    leftWheelSpeed=-1;
}
if (rightWheelSpeed<(-1.0)){
    rightWheelSpeed=1;

}

}

