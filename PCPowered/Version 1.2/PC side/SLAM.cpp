
#include <math.h>

#include "Communicator.h"

#include <zmq.h>

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <list>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>

#include "Position.hpp"
#include "Laser.hpp"
#include "Velocities.hpp"
#include "algorithms.hpp"



#include <opencv/cv.h>
#include <opencv/highgui.h>




using namespace std;
using namespace cv;

static const int POINTS_PER_SCAN		= 682;
static const int MAP_SIZE_PIXELS        = 800;
static const double MAP_SIZE_METERS     =  32;
static const int random_seed 			= 999;

bool loop = true;
mutex mtx;

class MinesURG04LX : public URG04LX
{
    
public:
    
    MinesURG04LX(void): URG04LX(
        70,          // detectionMargin
        145)         // offsetMillimeters
    {
    }
};

int coords2index(double x,  double y)
{    
    return y * MAP_SIZE_PIXELS + x;
}

void waitForKeyPress(){
	getchar();
	mtx.lock();
	loop = false;
	mtx.unlock();
}

int main(){
	thread t(waitForKeyPress);

	Communicator* comm = new Communicator(512, "10.42.0.13", 8001, "*", 8000);

	//get size of scan data
	char buf[4];
	comm->recv(buf, 4, 0);
	int bytesPerScan = atoi(buf);
	cout << bytesPerScan << endl;

	//allocate scan buffer
	int* scanT0 = (int*)malloc(bytesPerScan);
    int* scanT1 = (int*)malloc(bytesPerScan);
    int* scanT2 = (int*)malloc(bytesPerScan);
    int* scanT3 = (int*)malloc(bytesPerScan);

	//declare slam veriables
	MinesURG04LX laser;
	SinglePositionSLAM *slam = new RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed);
    Velocities v;

	//making a map
	Mat map(MAP_SIZE_PIXELS,MAP_SIZE_PIXELS,CV_8UC1);

    int x = 0, y = 0, yaw = 0;

    int loopCounter = 0;

    mtx.lock();
    while(loop)
    {
    	mtx.unlock();
    	//recv scan -- which includes recent optical flow data
    	memcpy(scanT3, scanT2, bytesPerScan);
        memcpy(scanT2, scanT1, bytesPerScan);
        memcpy(scanT1, scanT0, bytesPerScan);


        comm->recv(scanT0, bytesPerScan, 0);



        //first 4 values are odometry

    	if(x==0 && y==0 && yaw==0){
    		x = scanT0[0];
    		y = scanT0[1];
    		yaw = scanT0[2];
    	}

        if(abs(scanT0[2]) < 20){
            yaw = 0;
        } else {
            yaw = scanT0[2];
        }


        //forward velocity --> only x?? --> try this
        v.dxy_mm = (scanT0[0]-x);
        v.dtheta_degrees = (yaw)*0.01;
        v.dt_seconds = (scanT0[3])*0.001; //this is an approximation!!

    	x = scanT0[0]; y = scanT0[1];
    	yaw = scanT0[2];

        //printf("%f\n", v.dtheta_degrees);

        if(loopCounter > 4){
            slam->update(scanT2+sizeof(int)*4, v);
        	slam->getmap(map.data);
        }

    	imshow("MAP", map);
        if(waitKey(30) >= 0) break;

    	loopCounter++;

		mtx.lock();
	}

	t.join();

	//save the map
	char filename[100];
    sprintf(filename, "test.pgm");
    printf("\nSaving map to file %s\n", filename);
    
    FILE * output = fopen(filename, "wt");
    
    fprintf(output, "P2\n%d %d 255\n", MAP_SIZE_PIXELS, MAP_SIZE_PIXELS);
    
    for (int y=0; y<MAP_SIZE_PIXELS; y++)
    {
        for (int x=0; x<MAP_SIZE_PIXELS; x++)
        {
            fprintf(output, "%d ", map.data[coords2index(x, y)]);
        }
        fprintf(output, "\n");
    }

    printf("Done writing\n");

    printf("Delete scan\n");

    delete scanT0;
    delete scanT1;
    delete scanT2;
    delete scanT3;

    printf("Delete comm\n");

    delete comm;

    printf("Delete slam\n");

    delete slam;

    printf("Closing output\n");

    fclose(output);

    return 0;
}
