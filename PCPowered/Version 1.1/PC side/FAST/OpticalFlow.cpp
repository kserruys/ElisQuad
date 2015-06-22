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
	thread t(waitForKeyPress);

	list<CvPoint2D32f> opticalFlowVectors;

	//declare frames
	Mat frame1;
	Mat frame2;
	//time between frames is needed to get velocity
	high_resolution_clock::time_point firstFrame;
	high_resolution_clock::time_point secondFrame;
	duration<double> timeDiff;

	int iter = 524;

	//second get the image
	ostringstream fileName;
	fileName << "image" << iter <<".jpg";
	frame1 = imread(fileName.str(), CV_LOAD_IMAGE_GRAYSCALE);

	firstFrame = high_resolution_clock::now();

	//build pyramid for frame 1
	buildOpticalFlowPyramid(frame1, pyramid1, cvSize(pyrWindowSize,pyrWindowSize), 3);


	//start optical flow algorithm
	cout << "Started optical flow algorithm." << endl;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();


	double arrayX[4], arrayY[4];

    mtx.lock();
    while(iter<849)
    {
    	mtx.unlock();

    	//recv frame 2
		ostringstream fileName3;
		fileName3 << "image" << iter <<".jpg";
		frame2 = imread(fileName3.str(), CV_LOAD_IMAGE_GRAYSCALE);

		secondFrame = high_resolution_clock::now();
		timeDiff = duration_cast<duration<double>>(secondFrame - firstFrame);
		
		//detector->detect(frame1, KeyPointVector);
		//KeyPoint::convert(KeyPointVector, pointVector1);

	  	goodFeaturesToTrack(frame1, pointVector1, 200, 0.05, 1, Mat(),
	  		3, true, 0.04);

	  	
		//save picture with features to track
		/*Mat copy = frame1.clone();
		for(int i = 0; i < pintVector1.size(); i++){
			circle(copy, pointVector1[i], 4, Scalar(rng.uniform(0,255), rng.uniform(0,255),
             rng.uniform(0,255)), -1, 8, 0 ); 
		}
		
		ostringstream fileName;
		fileName << "flightphoto/image" << iter <<".jpg";
		imwrite(fileName.str(), frame1);*/
		

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

		//store pyramid 2 in pyramid 1
		frame1 = frame2.clone();
		pyramid1.swap(pyramid2);
		firstFrame = secondFrame;

		//find the average opticalVelocity
		opticalDisplacement = trajectoryCalc(pointVector1, pointVector2, featuresFound,
		featuresError, pointVector1.size());

		/*
		opticalVelocity.x = opticalDisplacement.x/timeDiff.count();
		opticalVelocity.y = opticalDisplacement.y/timeDiff.count();
		double avgX = arrayX[0]*0.2 + arrayX[1]*0.2 + arrayX[2]*0.2 + arrayX[3]*0.2 + opticalVelocity.x*0.2;
		double avgY = arrayY[0]*0.2 + arrayY[1]*0.2 + arrayY[2]*0.2 + arrayY[3]*0.2 + opticalVelocity.y*0.2;
		*/


		opticalFlowVectors.push_back(opticalDisplacement);
		iter++;
		mtx.lock();
	}
	
	t.join();

	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
	double fps = ((double)opticalFlowVectors.size())/time_span.count();

	ofstream myFile;
	myFile.open ("opticalFlow.txt");

	myFile << "FPS: \t" << fps << endl;

	iter=0;

	for (list<CvPoint2D32f>::iterator it=opticalFlowVectors.begin(); it!=opticalFlowVectors.end(); ++it){
		  myFile << it->x << "\t" << it->y << endl;
	}

  	myFile.close();		

}
