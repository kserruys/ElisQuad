#include "Communicator.h"

#include <zmq.h>

#include <math.h>
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
#include "WheeledRobot.hpp"
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

	Communicator* comm = new Communicator(512, "10.42.0.13", 9002, "*", 9000);

	//get size of scan data
	char buf[4];
	comm->recv(buf, 4, 0);
	int bytesPerScan = atoi(buf);
	cout << bytesPerScan << endl;

	//allocate scan buffer
	int* scan = (int*)malloc(bytesPerScan);

	//declare slam veriables
	MinesURG04LX laser;
	SinglePositionSLAM *slam = new RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed);

	//making a map
	Mat map(MAP_SIZE_PIXELS,MAP_SIZE_PIXELS,CV_8UC1);

    mtx.lock();
    while(loop)
    {
    	mtx.unlock();
    	//recv scan -- which includes recent optical flow data
    	comm->recv(scan, bytesPerScan, 0);

    	slam->update(scan);
    	slam->getmap(map.data);

    	imshow("MAP", map);
        if(waitKey(30) >= 0) break;

    	
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

    delete scan;

    printf("Delete comm\n");

    delete comm;

    printf("Delete slam\n");

    delete slam;

    printf("Closing output\n");

    fclose(output);

    return 0;
}