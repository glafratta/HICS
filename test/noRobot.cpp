//#include "Box2D/Box2D.h"
#include "../src/configurator.h"
#include <thread>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip> 
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <ncurses.h>
#include <ctime>
#include <dirent.h>
#include <filesystem>
#include "CppTimer.h"

class DataInterface {
public:
int iteration = 0;
Configurator * box2d;
char * folder;
    DataInterface(Configurator * _box2d, char * _folder){ //reads data from folder
		box2d = _box2d;
		std::filesystem::path folderPath(_folder);
		if (exists(folderPath)){
			if (is_directory(folderPath)){
				folder = _folder;
                box2d->folder = _folder;
			}
			else{
				printf("not a directory");
			}
		}
		else{
			printf("%s doesn't exist", _folder);
		}


	}

	bool newScanAvail(){ //uncomment sections to write x and y to files		
        iteration++;
		char filePath[256];
        char folderName[256];
        sprintf(folderName,"%s", folder);
		sprintf(filePath, "%smap%04d.dat", folderName, iteration);
        printf("%s\n", filePath);
        FILE *f;
        if (!(f=fopen(filePath, "r"))){
            return false;
        }
        else {
            fclose(f);
        }
        //std::vector <cv::Point2f> current;
        std::vector <Point> data;
		std::ifstream file(filePath);

        float x, y, x2, y2;
        Point p1, p2_1;
		//box2d->previous = box2d->current;
		//box2d->current.clear();
		while (file>>x>>y){
            x = round(x*1000)/1000;
			y = round(y*1000)/1000;
            x2 = round(x*100)/100;
			y2 = round(y*100)/100;
            Point p(x,y);
            Point p2(x2,y2);
            //Point pp = *(&p-1);
            if (p!=p1){
                data.push_back(p);
            	//current.push_back(cv::Point2f(x, y));
            }
            //if (p2.r<1.0 && p2!=*(&p2-1)){
            if (p2.r<1.0 && p2!=p2_1){
                box2d->currentBox2D.push_back(p2);
            }
			p1=p;
            p2_1 = p2;
		}
		file.close();
       // printf("current: %i, previous: %i\n", box2d->current.size(), box2d->previous.size());
        box2d->NewScan(data);
        return true;
		


	}
};

class StepCallback{
    float L,R;
    Configurator * box2d;
public:
    StepCallback(Configurator * c): box2d(c){}
    void step(){
        L=box2d->getDMP()->getAction().getLWheelSpeed();
        R= box2d->getDMP()->getAction().getRWheelSpeed();
        printf("step :R= %f, L = %f\n,", R, L);

    }
};


//FOR THREAD DEBUGGING

class TimerStep: public CppTimer{
    StepCallback sc;
    public:
    TimerStep(StepCallback & _step): sc(_step){}
    void timerEvent(){
        sc.step();
    }
};

class TimerDI: public CppTimer{
    DataInterface di;
    public:
    TimerDI(DataInterface & _di): di(_di){}
    void timerEvent(){
        if (!di.newScanAvail()){
            this->stop();
        }
    }
};

//END THREAD DEBUGGING

//argv: 1. directory to open 2. timeoff (0=timeron) 3. planning on 4. debug on

int main(int argc, char** argv) {
    //BENCHMARK
//	auto begin = std::chrono::high_resolution_clock::now();


    //HOW MANY FILES IN DIRECTORY
    DIR *dp;
    int i = 0;
    struct dirent *ep;     
    dp = opendir(argv[1]);

    if (dp != NULL)
    {
        while (ep = readdir(dp)){
            i++;
        }
        closedir(dp);
    }
    else{
        printf("Couldn't open the directory%s", argv[1]);
    }


    //FILE * file;
    char filePrefix[5];
    sprintf(filePrefix, "map");
    int fileCount =1;
    // bool fileExists = true;
    // char fileName[250];
    // sprintf(fileName, "%s%s%04i.dat", argv[2], filePrefix, fileCount);
   // std::ifstream file(fileName);
   char file[256];
   FILE *f;
    while ( sprintf(file, "%s%s%04i.dat", argv[1], filePrefix, fileCount)>-1 &&(f = fopen(file, "r"))!=NULL){   
        //printf("* ");
        //file.close();
        fileCount++;
        memset(file, 0, sizeof(file));
        //sprintf(file, "%s%s%04i.dat", argv[1], filePrefix, fileCount);
        //sprintf(fileName, "%s%s%04i.dat", argv[2], filePrefix, fileCount);
        //file = std::ifstream(fileName);
        fclose(f);
    }
    printf("\n%i files\n", fileCount);



    //CREATE A FOLDER TO STORE THE RESULTS FOR THIS RUN

    // //TODAYS DATE AND TIME
    // time_t now =time(0);
    // tm *ltm = localtime(&now);
    // int y,m,d;
    // y=ltm->tm_year-100;
    // m = ltm->tm_mon +1;
    // d=ltm->tm_mday;
    
    // //PICK DIRECTORY NAME
    // char dirName[256];
    // int copynumber=1;
    // while (true){
    //     DIR *dir; 
    //     char tmp[256];
    //     sprintf(tmp, "LoRNoTimer%02i-%02i-%02i_%i",d, m, y, copynumber);
    //     dir = opendir(tmp);
    //     if (dir ==NULL){
    //         sprintf(dirName, "%s", tmp);
    //         std::filesystem::create_directory(dirName);
    //         break;
    //     } //if the directory DOESN'T return a void pointer (= exists already)
    //     else if (dir!=NULL){
    //         copynumber++;                                                           //increase the number at the end of the name
    //         closedir(dir);
    //     }

    // }
                                //should exit when dir is null pointer

    //DATA INTERCFACE
    bool timerOff=atoi(argv[2]);
    Primitive desiredDMP;
    bool debug =1;
    if (argc>4){
        debug = atoi(argv[4]);
    }
    Configurator box2d(desiredDMP, debug, timerOff);
    box2d.setReadMap("map");
    box2d.setFolder(argv[1]);   
    if (argc >3){
        box2d.planning = atoi(argv[3]);
        printf("planning = %i\n", box2d.planning);
    }     
    StepCallback cb(&box2d);
    DataInterface dataInterface(&box2d, argv[1]);

                //iterate through files

    if (timerOff ==0){
        TimerDI lidar(dataInterface);
        TimerStep motors(cb);
        lidar.startms(200);
        motors.startms(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(200*fileCount)); //simulates lidar
        lidar.stop();
        motors.stop();
    }
    else if (timerOff){
                // b2Vec2 velocity = {0,0};
        //for (int i=0; i<40;i++){
          while  (dataInterface.newScanAvail()){
            cb.step();

        }
    }


    printf("total time = %.2f", box2d.totalTime);

    // FINISH BENCHMARKING
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    // printf("scanned in %f seconds\n", elapsed.count()* 1e-9);




}


