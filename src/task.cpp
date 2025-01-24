#include "task.h"


simResult Task::willCollide(b2World & _world, int iteration, bool debugOn, float remaining, float simulationStep){ //CLOSED LOOP CONTROL, og return simreult
		simResult result=simResult(simResult::resultType::successful);
		result.endPose = start;
		if (action.L==0 & action.R==0){
			return result;
		}
		Robot robot(&_world);
		Listener listener;
		_world.SetContactListener(&listener);	
		FILE * robotPath;
		if (debugOn){
			sprintf(planFile, "/tmp/robot%04i.txt", iteration);
			robotPath = fopen(planFile, "a");
		}
		float theta = start.q.GetAngle();
		b2Vec2 instVelocity = {0,0};
		robot.body->SetTransform(start.p, theta);
		int stepb2d=0;
		float traj_error=0;
		for (stepb2d; stepb2d < (HZ*remaining); stepb2d++) {//3 second
			// if (direction==DEFAULT){
			// 	traj_error=remainder(robot.body->GetTransform().q.GetAngle(), M_PI_2);
			// 	correct.update(traj_error);
			// 	correct(action, stepb2d);
			// 	printf("angle =%f,error =%f\n", robot.body->GetTransform().q.GetAngle(),traj_error);
			// }
			instVelocity.x = action.getLinearSpeed()*cos(theta);
			instVelocity.y = action.getLinearSpeed()*sin(theta);
			robot.body->SetLinearVelocity(instVelocity);
			robot.body->SetAngularVelocity(action.getOmega());
			robot.body->SetTransform(robot.body->GetPosition(), theta);
			if (debugOn){
				fprintf(robotPath, "%f\t%f\n", robot.body->GetPosition().x, robot.body->GetPosition().y); //save predictions/
			}
			bool out_ty= robot.body->GetTransform().p.y>=BOX2DRANGE;
			bool out_by= robot.body->GetTransform().p.y<=-BOX2DRANGE;
			bool out_tx= robot.body->GetTransform().p.x>=BOX2DRANGE;
			bool out_bx= robot.body->GetTransform().p.x<=-BOX2DRANGE;
			bool out= out_bx||out_by||out_tx|| out_ty;
			if (checkEnded(robot.body->GetTransform(), direction).ended || (start.p-robot.body->GetTransform().p).Length()>=simulationStep||out){
				break;
			}
			_world.Step(1.0f/HZ, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
			theta += action.getOmega()/HZ; //= omega *t
			if (listener.collisions.size()>0){ //
				int index = int(listener.collisions.size()/2);
				Disturbance collision = Disturbance(listener.collisions[index]);
				//b2Vec2 distance = collision.getPosition()-robot.body->GetTransform().p;
				result = simResult(simResult::resultType::crashed, collision);
				stepb2d=0;
				break;
			}
		}
		result.endPose = robot.body->GetTransform();
		result.step=stepb2d;
		for (b2Body * b = _world.GetBodyList(); b!=NULL; b = b->GetNext()){
			_world.DestroyBody(b);
		}
		if (debugOn){
			fclose(robotPath);
		}
		int bodies = _world.GetBodyCount();
		return result;
	
}


void Task::Correct::operator()(Action & action, int step){
	float tolerance = 0.1; //tolerance in radians/pi = just under 2 degrees degrees
	if (action.getOmega()!=0){ //only check every 2 sec, og || motorstep<1
		printf("returning\n");
		return;
	}
	//printf("error buffer sum = %f, i=%f\n", p(), get_i());
	if (fabs(get_i())>tolerance){//& step>correction_rate
		float p_correction= ((p()/bufferSize)*kp)/2; //do not increase one wheel speed too much
		float i_correction= (get_i()*ki)/2; //do not increase one wheel speed too much
		float d_correction= (get_d()*kd)/2; //do not increase one wheel speed too much
			action.R -= p_correction+ i_correction; 
		 	action.L+= p_correction+ i_correction;
		if (action.L>1.0){
		action.L=1.0;
		}
		if (action.R>1.0){
			action.R=1;
		}
		if (action.L<(-1.0)){
			action.L=-1;
		}
		if (action.R<(-1.0)){
			action.R=-1;
		}
	
	}
}

float Task::Correct::errorCalc(Action a, double x){
	float result=0;
	if (a.getOmega()!=0){
		return result;
	}
	else{
		return sin(a.getOmega())*MOTOR_CALLBACK-float(x); //-ve error if robot goes R, +ve error if goes L
	}	

}


float Task::Correct::update(float e){
	float p0=p();
	p_buffer.erase(p_buffer.begin());
	p_buffer.push_back(e);
	float p1=p();
	mf.buffer.erase(mf.buffer.begin());
	mf.buffer.push_back(p1);
	d=p1-p0;
	i+=e;
	return p1;
}


Direction Task::H(Disturbance ob, Direction d, bool topDown){
	if (ob.isValid()){
        if (ob.getAffIndex()==int(InnateAffordances::AVOID)){ //REACTIVE BEHAVIOUR
            if (d == Direction::DEFAULT & !topDown){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<0){//angle formed with robot at last safe pose
                    d= Direction::LEFT; //go left
                }
                else if (ob.getAngle(start)>0){ //angle formed with robot at last safe pose
                    d= Direction::RIGHT; //
                }   
                else{
                    int c = rand() % 2;
                    d = static_cast<Direction>(c);

                }
            }
        }
		else if (ob.getAffIndex()==int(InnateAffordances::PURSUE)){
			if (d == Direction::DEFAULT & !topDown){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<-.1){//angle formed with robot at last safe pose
                    d= Direction::RIGHT; //go left
                }
                else if (ob.getAngle(start)>0.1){ //angle formed with robot at last safe pose, around .1 rad tolerance
                    d= Direction::LEFT; //
                }   
            }
		}
}
    return d;
}

void Task::setEndCriteria(Angle angle, Distance distance){
	switch(disturbance.getAffIndex()){
		case PURSUE:{
			endCriteria.angle=Angle(0);
			endCriteria.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
		}
		break;
		default:
		endCriteria.distance = distance;
		endCriteria.angle = angle;
		break;
	}
	if (disturbance.isValid()){
		endCriteria.valid_d=true;
	}
}

EndedResult Task::checkEnded(b2Transform robotTransform, Direction dir,bool relax, std::pair<bool,b2Transform> use_start){ //self-ended
	if (dir==UNDEFINED){
		dir=direction;
	}
	b2Transform this_start= start;
	if (!use_start.first){
		this_start=use_start.second;
	}
	EndedResult r;
	Angle a;
	Distance d;
	//printf("check ended\n");
	b2Vec2 distance=this_start.p-robotTransform.p;
	if (round(distance.Length()*100)/100>=BOX2DRANGE){ //if length reached or turn
		if (debug_k){
		//	printf("distance of %f exceeds range, ended\n", distance.Length());
		}
		r.ended =true;
	}
	if (disturbance.isValid()){
		b2Vec2 v = disturbance.getPosition() - robotTransform.p; //distance between disturbance and robot
		d= Distance(v.Length());
		if (action.getOmega()!=0){	
			float angleL = start.q.GetAngle()+SAFE_ANGLE;
			float angleR = start.q.GetAngle()-SAFE_ANGLE;
			float robotAngle=robotTransform.q.GetAngle();
			int mult= start.q.GetAngle()/(3*M_PI_4);
			if (mult>0){
				if (dir==LEFT& robotAngle<0){
					robotAngle+=2*M_PI;
				}
			}
			else if (mult<0){
				if (dir==RIGHT & robotAngle>0){
					robotAngle-=2*M_PI;
				}
			}

			bool finishedLeft=robotAngle>=angleL;//-(action.getOmega()*HZ)/2;
			bool finishedRight=robotAngle<=angleR;//+(action.getOmega()*HZ)/2;
			if (finishedLeft|| finishedRight){
				if (disturbance.getAffIndex()==AVOID){
					disturbance.invalidate();
				}
				if (debug_k){
			//	printf("angle of %f exceeds range, ended\n", robotTransform.q.GetAngle());
				}
				r.ended = 1;
			}
		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			a =Angle(robotTransform.q.GetAngle());
			if (debug_k){
			//	printf("control goal null has D, ended\n");
			}			
			r.ended = true;
		}
		else if (getAffIndex()==int(InnateAffordances::PURSUE)){
			a = Angle(disturbance.getAngle(robotTransform));
			
			if (relax){
				Distance _d(RELAXED_DIST_ERROR_TOLERANCE);
				r.ended = d<=_d; 
			}
			else{
				r.ended = d<=endCriteria.distance; 
			}
			//if (!r.ended){
			//}
			
			if (debug_k){
				printf("distance to goal=%f, endCriteria.distance =%f", d.get(), endCriteria.distance.get());
			}
			if (debug_k & r.ended){
				printf("robot %f %f has reached goal, ended =%i\n", robotTransform.p.x, robotTransform.p.y);
			}	
		}
	}
	else if (dir==LEFT || dir ==RIGHT){
		float angleL = this_start.q.GetAngle()+endCriteria.angle.get();
		float angleR = this_start.q.GetAngle()-endCriteria.angle.get();
		r.ended = (robotTransform.q.GetAngle()>=angleL || robotTransform.q.GetAngle()<=angleR);
		if (debug_k &r.ended){
			//printf("turn done, but no goal\n");
		}	
	}
	r.estimatedCost = endCriteria.getStandardError(a,d);
	return r;

}

EndedResult Task::checkEnded(State n,  Direction dir, bool relax, std::pair<bool,b2Transform> use_start){ //check error of node compared to the present Task
	EndedResult r;
	Angle a;
	Distance d;
	r = checkEnded(n.endPose, dir, relax, use_start);
	r.estimatedCost+= endCriteria.getStandardError(a,d, n);
	return r;
}

EndCriteria Task::getEndCriteria(const Disturbance &d){
	EndCriteria endCriteria;
	switch(disturbance.getAffIndex()){
	case PURSUE:{
		endCriteria.angle=Angle(0);
		endCriteria.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
	}
	break;
	default:
	endCriteria.distance = BOX2DRANGE;
	break;
}
}


