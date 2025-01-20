#include "configurator.h"
#include "libcam2opencv.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include "Iir.h"
//#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES

void get_Foldername(char* custom, char name[60]){
    time_t now =time(0);
	tm *ltm = localtime(&now);
	int y,m,d, h, min;
	y=ltm->tm_year-100;
	m = ltm->tm_mon +1;
	d=ltm->tm_mday;
	h= ltm->tm_hour;
	min = ltm->tm_min;
	sprintf(name, "%s_%02i%02i%02i_%02i%02i",custom, d,m,y,h,min);
}

void forget(Configurator*);

Disturbance set_target(int&, b2Transform);

std::vector <BodyFeatures> WorldBuilder::processData(const CoordinateContainer& points, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <Pointf> ptset= set2vec(points);
    std::pair<bool,BodyFeatures> feature= getOneFeature(ptset);
    if (feature.first){
        feature.second.pose.q.Set(start.q.GetAngle());
        result.push_back(feature.second);
    }
    return result;
}


class LidarInterface : public A1Lidar::DataInterface{
ConfiguratorInterface * ci;
public: 
    int mapCount =0;

    LidarInterface(ConfiguratorInterface * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (ci == NULL){
			printf("null pointer to ci\n");
			return;
		}
		//ci->data.clear();
		ci->data2fp.clear();
		mapCount++;
		Pointf p, p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		if (ci->debugOn){
			f=fopen(name, "w");
		}
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				p2f=Pointf(x2, y2);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		if (ci->debugOn){
		fclose(f);
		}
		if (!ci->data2fp.empty()){
			ci->setReady(1);
		}
		ci->iteration++;

	}


};

class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    float L=0;
	float R=0;
public:
int ogStep=0;
Configurator * c;
int run=0;

MotorCallback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	printf("graph size=%i\n", c->transitionSystem.m_vertices.size());
	if (c->getIteration() <=0){
		return;
	}
	if (!c->running){
		motors.setRightWheelSpeed(0);
 	   motors.setLeftWheelSpeed(0);		
	}

	c->trackTaskExecution(*c->getTask());
	EndedResult er = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, false);
	//bool planEnded = c->getTask()->motorStep<1 & c->planVertices.empty() & c->transitionSystem[c->currentEdge].direction!=STOP;
	//EndedResult er2 = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, true);
	if (er.ended ){ //|| (er2.ended & c->getTask()->motorStep<1 & c->planVertices.empty())
		printf("goal reached\n");
		Disturbance new_goal=set_target(run, c->controlGoal.start);
		c->controlGoal = Task(new_goal, UNDEFINED);
		b2Vec2 v = c->controlGoal.disturbance.getPosition() - b2Vec2(0,0);
		printf("new control goal start: %f, %f, %f, distance = %f, valid =%i\n", c->controlGoal.start.p.x, c->controlGoal.start.p.y, c->controlGoal.start.q.GetAngle(), v.Length(), c->controlGoal.disturbance.isValid());
		if (c->is_benchmarking()){
			FILE * f = fopen(c->statFile, "a+");
			fprintf(f, "!");
			fclose(f);			
		}
		forget(c);

	}
	c->planVertices = c->changeTask(c->getTask()->change,  ogStep, c->planVertices);
	R= c->getTask()->getAction().getRWheelSpeed();
	L=c->getTask()->getAction().getLWheelSpeed();
	if (c->getTask()->direction==LEFT){
		R*=1.33;
		L*=1.33*1.05;
	}
	else if (c->getTask()->direction==RIGHT){
		R*=1.17;
		L*=1.17;
	}
	else if (c->getTask()->direction==DEFAULT){
		R*=1.1;
		L*=1.1;
	}
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	//printf("og step: %i ,R=%f\tL=%f, vertex=%i\n", ogStep, c->getTask()->getAction().getRWheelSpeed(), c->getTask()->getAction().getLWheelSpeed(), c->currentVertex);
}
};

struct CameraCallback: Libcam2OpenCV::Callback {
    char dumpname[50];
    double signal=0;
    double filtered_signal=0;
    Iir::Butterworth::LowPass<order>low_pass;
    Iir::Butterworth::BandStop<order>band_stop;
	const int reset_hz=10;

    CameraCallback(MotorCallback * _cb):cb(_cb){
        low_pass.setup(FPS, cutoff_frequency);
        band_stop.setup(FPS, DC, band_width);
    }


	void hasFrame(const cv::Mat &frame, const libcamera::ControlList &){
		if (cb==NULL){
			printf("null cb\n");
		}
		if (cb->c==NULL){
			printf("null c\n");
		}
		if (cb->c->getTask()==NULL){
			printf("null task\n");
		}
        float error=0;
        cv::Vec2d  optic_flow=imgProc.avgOpticFlow(frame);
        cv::Vec2d  optic_flow_filtered=optic_flow;
        signal= signal+optic_flow[0];
        optic_flow_filtered[0]=low_pass.filter((optic_flow[0]));
        optic_flow_filtered[0]= band_stop.filter(optic_flow_filtered[0]);
        filtered_signal=filtered_signal+optic_flow_filtered[0];
		if (cb->c->getTask()->motorStep!=cb->ogStep & cb->c->getTask()->motorStep!=0){ //, in the future t.motorStepdiscard will be t.change
																//signal while the robot isn' moving
        	Task::Action action= cb->c->getTask()->getAction();
			error= cb->c->getTask()->correct.errorCalc(action, double(optic_flow_filtered[0]));
		}
        cb->c->getTask()->correct.update(error); //for now just going straight
    }
private:
ImgProc imgProc;
MotorCallback *cb=NULL;
};

float Configurator::taskRotationError(){
    return 0;
}
