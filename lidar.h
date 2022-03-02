#include "../rplidar_rpi/a1lidarrpi.h"
//include "/usr/include/pigpio.h"
//#include "map.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>
#include <sstream> //for writing string into file, for checking, std::ostringstream
//#define title(name, number,) name##number
#include <iostream>

class DataInterface : public A1Lidar::DataInterface{
public: 
    int numberOfScans=0; //for testing purposes
    Box2DEnv * box2d;
    

    void setBox2D(Box2DEnv * _box2d){
        box2d = _box2d;
    }


	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //should print map name and coordinates to the console
        numberOfScans++;
        box2d->createMap();
        Map * map = box2d->maps->at(box2d->maps->size()-1);
        std::cout<<"map in box2d->maps\n";
        std::stringstream tmp;
        tmp << "map" << std::setw(4) << std::setfill('0') << numberOfScans << ".dat";
        const char * filename = tmp.str().c_str();
        //std::cout<<filename<<std::endl;
        FILE * file =fopen(filename, "w+"); //uncomment
		for (A1LidarData &data:data){
			if (data.valid){
                std::ostringstream stream;
                Obstacle* obstacle = new Obstacle(*(box2d->world), approximate(data.x), approximate(data.y)); 
                obstacle->body->SetAwake(true);
			    map->obstacles->push_back(obstacle);
                //stream <<obstacle->body->GetPosition().x<<"\t"<<obstacle->body->GetPosition().y<<"\n";
                std::cout<<obstacle->body->GetPosition().x<<"\t"<<obstacle->body->GetPosition().y<<"\n";
//                const char * line = stream.str().c_str();
                // fputs(line, file);
                // fclose(file);
                // printf("hi");
			}
		}
        //box2d->simulate(); //simulates collision for 5 seconds

	}

    // std::string getFilename(){
    //     char numbers[3];
    //     for (int i=2; i>=0; i--){
    //         if (number[i]<9){
    //             number[i]++;
    //             itoa(number[i], numbers, 10);
    //             goto finish;
    //         }
    //         else if (number[i]==9){
    //             number[i]=0;
    //             itoa(number[i], numbers, 10);
    //         }
    //     }
    //     finish:
    //     return "map"+numbers+".dat"; //NOT TESTED
    // }


	float approximate(float a){
		a = round(a*10)/10;
		if (a<0){
			a-=0.05;
		}
		else if (a>=0){
			a+= 0.05;
		}
		return a;
	}
};

