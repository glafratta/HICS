
#include "custom.h"


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning=1;
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
		printf("debug = %i\n", atoi(argv[1]));
	}
	if (argc>2){
		configurator.simulationStep = atof(argv[2]);
	}
	printf("TARGETLESS PLANNING, hz=%f, turn f=%f yallaaaaa", HZ, TURN_FRICTION);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	MotorCallback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
