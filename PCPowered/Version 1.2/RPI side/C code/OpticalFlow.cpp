#include "Communicator.h"

#include <zmq.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
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
#include <ctime>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;

int frameWidth = 320;
int frameHeight = 240;

bool loop = true;
mutex mtx;

int FASTThreshold = 40, pyrWindowSize = 50;
vector<KeyPoint> KeyPointVector;
vector<Mat> pyramid1, pyramid2;
vector<Point2f> pointVector1, pointVector2;
CvPoint2D32f opticalVelocity;
CvPoint2D32f opticalDisplacement;
vector<uchar> featuresFound;
vector<float> featuresError;

RNG rng(12345);

void waitForKeyPress(){
	getchar();
	mtx.lock();
	loop = false;
	mtx.unlock();
}

static CvPoint2D32f trajectoryCalc(vector<Point2f> pointVector1, vector<Point2f> pointVector2,
 vector<uchar> featuresFound, vector<float> featuresError, int featureNumber)
{
	CvPoint2D32f displacement;
	double dx = 0; double dy = 0; int goodPoints = 0;

	for(int i=0; i < featureNumber; i++)
	{

		if(featuresFound[i]==0 || featuresError[i]>20)
		{
			continue;
		}

		double tdx = (double)((pointVector1[i].x) - (pointVector2[i].x));
		double tdy = (double)((pointVector1[i].y) - (pointVector2[i].y));

		if(((opticalDisplacement.x-30<tdx && tdx<opticalDisplacement.x+30) && (opticalDisplacement.y-30<tdy && tdy<opticalDisplacement.y+30))
		 || opticalDisplacement.x==0 || opticalDisplacement.y==0){
			dx += tdx;
			dy += tdy;
			goodPoints++;
		}
	}

	if(goodPoints != 0)
	{
		displacement = cvPoint2D32f((dx/(double)goodPoints), (dy/(double)goodPoints));
	}
	else
	{
		displacement = cvPoint2D32f(0,0);
	}


	return displacement;
}


int main(){
	list<CvPoint2D32f> opticalFlowVectors;

	//set up communicator	
	//Communicator* comm = new Communicator(512, "10.42.0.13", 9002, "*", 9000);


	//initialize camera
    VideoCapture cap(0);
    if(!cap.isOpened()){
        cout << "No camera found." << endl;
        return -1;
    }

	cap.set(CV_CAP_PROP_FRAME_WIDTH,frameWidth);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,frameHeight);
    cap.set(CV_CAP_PROP_FPS, 30);

    //initialize frames
    Mat frame;
	Mat frame1(frameHeight,frameWidth,CV_8UC1);
	Mat frame2(frameHeight,frameWidth,CV_8UC1);

	//time between frames is needed to get velocity
	high_resolution_clock::time_point firstFrame;
	high_resolution_clock::time_point secondFrame;
	duration<double> timeDiff;

	cout << "getting first image." << endl;

	//second get the image
	cap >> frame;
	cout << "got first image." << endl;

	cvtColor(frame, frame1, CV_BGR2GRAY);
	cout << "converted to gray image" << endl;

	firstFrame = high_resolution_clock::now();

	//build pyramid for frame 1
	buildOpticalFlowPyramid(frame1, pyramid1, cvSize(pyrWindowSize,pyrWindowSize), 3);


	//start optical flow algorithm
	cout << "Started optical flow algorithm." << endl;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	int iter = 0;

	double arrayX[4], arrayY[4];

	while(1)
    {
    	//recv frame 2
		cap >> frame;
		cvtColor(frame, frame2, CV_BGR2GRAY);
		//delete frame
		frame.release();

		secondFrame = high_resolution_clock::now();
		timeDiff = duration_cast<duration<double>>(secondFrame - firstFrame);
		
		//detector->detect(frame1, KeyPointVector);
		//KeyPoint::convert(KeyPointVector, pointVector1);

	  	goodFeaturesToTrack(frame1, pointVector1, 200, 0.01, 1, Mat(),
	  		3, false, 0.04);

	  	/*
		//save picture with features to track
		Mat copy = frame1.clone();
		for(int i = 0; i < pointVector1.size(); i++){
			circle(copy, pointVector1[i], 4, Scalar(rng.uniform(0,255), rng.uniform(0,255),
             rng.uniform(0,255)), -1, 8, 0 ); 
		}
		
		ostringstream fileName;
		fileName << "flightphoto/image" << iter <<".jpg";
		imwrite(fileName.str(), frame1);
		*/

		/*
		if(KeyPointVector.size() > 30)
			FASTThreshold++;
		else 
			FASTThreshold--;
		*/

		//build pyramid for frame 2
		buildOpticalFlowPyramid(frame2, pyramid2, cvSize(pyrWindowSize, pyrWindowSize), 3);


		//run Lucas Kanade optical flow if features have been found
		if(pointVector1.size() > 0)
		{
			calcOpticalFlowPyrLK(pyramid1, pyramid2, pointVector1,
			 pointVector2, featuresFound, featuresError,
			 cvSize(pyrWindowSize, pyrWindowSize), 0,
			 cvTermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 0.2),
			 0,0.0001);
		}

		/* //if pics with flow vectors are wanted
		Mat frame3;
		cvtColor(frame2, frame3, CV_GRAY2RGB);

		for(int i=0; i < pointVector1.size(); i++){
			if(featuresFound[i]==0 || featuresError[i]>50)
			{
				//printf("Error is: %f\n",featuresError[i]);
				
			} else {
				CvPoint p0 = cvPoint(
						cvRound(pointVector1[i].x),
						cvRound(pointVector1[i].y));

				CvPoint p1 = cvPoint(
						cvRound(pointVector2[i].x),
						cvRound(pointVector2[i].y));

				line(frame3, p0, p1, CV_RGB(255,0,0), 1, 8, 0);
			}
		}

		ostringstream fileName2;
		fileName2 << "flightphoto/flow" << iter <<".jpg";
		imwrite(fileName2.str(), frame3);
		*/

		//store pyramid 2 in pyramid 1
		//first delete frame1
		frame1.release();
		//clone frame2
		frame1 = frame2.clone();
		pyramid1.swap(pyramid2);
		firstFrame = secondFrame;
		//delete frame2
		frame2.release();

		//find the average opticalVelocity
		opticalDisplacement = trajectoryCalc(pointVector1, pointVector2, featuresFound,
		featuresError, pointVector1.size());

		cout << "loop " << iter << endl;

		//if speed is required
		/*
		opticalVelocity.x = opticalDisplacement.x/timeDiff.count();
		opticalVelocity.y = opticalDisplacement.y/timeDiff.count();
		*/

		/*
		double avgX = arrayX[0]*0.2 + arrayX[1]*0.2 + arrayX[2]*0.2 + arrayX[3]*0.2 + opticalVelocity.x*0.2;
		double avgY = arrayY[0]*0.2 + arrayY[1]*0.2 + arrayY[2]*0.2 + arrayY[3]*0.2 + opticalVelocity.y*0.2;

		char xBuf[7]; char yBuf[7];
		//avgX must be sent negative because the camera is reversly mounted on the quad.
		int xBufLen = sprintf(xBuf, "%.1f", avgX);
		int yBufLen = sprintf(yBuf, "%.1f", avgY);
		comm->send(xBuf,xBufLen,ZMQ_NOBLOCK);
		comm->send(yBuf,yBufLen,ZMQ_NOBLOCK);
		*/

		opticalFlowVectors.push_back(opticalDisplacement);
		iter++;
		mtx.lock();
	}
}