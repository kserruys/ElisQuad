#include "Communicator.h"
#include "quicklz.h"

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
CvPoint displacement;
vector<uchar> featuresFound;
vector<float> featuresError;

void waitForKeyPress(){
	getchar();
	mtx.lock();
	loop = false;
	mtx.unlock();
}

static CvPoint trajectoryCalc(vector<Point2f> pointVector1, vector<Point2f> pointVector2,
 vector<uchar> featuresFound, vector<float> featuresError, int featureNumber)
{
	CvPoint displacement;
	int dx = 0; int dy = 0; int goodPoints = 0;

	for(int i=0; i < featureNumber; i++)
	{
		if(featuresFound[i]==0 || featuresError[i]>20)
		{
			continue;
		}
		dx += cvRound(pointVector1[i].x) - cvRound(pointVector2[i].x);
		dy += cvRound(pointVector1[i].y) - cvRound(pointVector2[i].y);
		goodPoints++;
	}

	if(goodPoints != 0)
	{
		displacement = cvPoint(round(dx/goodPoints), round(dy/goodPoints));
	}
	else
	{
		displacement = cvPoint(0,0);;
	}


	return displacement;
}


int main(){
	thread t(waitForKeyPress);

	list<CvPoint> opticalFlowVectors;

	//set up communicator	
	Communicator* comm = new Communicator(512, "192.168.2.3", 9002, "*", 9000);

	//receive size of one image
	char frameWidthBuf[3];
	char frameHeightBuf[3];
	comm->recv(frameWidthBuf, 3, 0);
	comm->recv(frameHeightBuf, 3, 0);
	//extract data
	int frameWidth = atoi(frameWidthBuf);
	int frameHeight = atoi(frameHeightBuf);
	int frameSize = frameWidth*frameHeight;

	//declare frames
	Mat frame1(frameWidth,frameHeight,CV_8UC1);
	Mat frame2(frameWidth,frameHeight,CV_8UC1);

	//second recv the first frame

	//recv the size of the encoded frame
	char encSizeBuf[6];
	comm->recv(encSizeBuf, 6, 0);
	int encSize = atoi(encSizeBuf);

	vector<uchar> enc = vector<uchar>(encSize);

	//recv the encoded frame
	comm->recv(&enc[0], encSize, 0);

	//decode the frame
	qlz_state_decompress *state_decompress = (qlz_state_decompress *)malloc(sizeof(qlz_state_decompress));
	qlz_decompress((const char*) &enc[0], (char*) frame1.data, state_decompress);


	//build pyramid for frame 1
	buildOpticalFlowPyramid(frame1, pyramid1, cvSize(pyrWindowSize,pyrWindowSize), 3);


	//start optical flow algorithm
	cout << "Started optical flow algorithm." << endl;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
    mtx.lock();
    while(loop)
    {
    	mtx.unlock();

    	//recv frame 2		
		//recv the size of the encoded frame
		comm->recv(encSizeBuf, 6, 0);
		encSize = atoi(encSizeBuf);

		enc = vector<uchar>(encSize);

		//recv the encoded frame
		comm->recv(&enc[0], encSize, 0);

		//uncompress recv data
		qlz_decompress((const char*) &enc[0], (char*) frame2.data, state_decompress);
		

		FeatureDetector* detector = new FastFeatureDetector(FASTThreshold,true);
		detector->detect(frame1, KeyPointVector);
		delete detector;

		if(KeyPointVector.size() > 30)
			FASTThreshold++;
		else 
			FASTThreshold--;

		//build pyramid for frame 2
		buildOpticalFlowPyramid(frame2, pyramid2, cvSize(pyrWindowSize,pyrWindowSize), 3);
		KeyPoint::convert(KeyPointVector, pointVector1);

		//run Lucas Kanade optical flow if features have been found
		if(KeyPointVector.size() > 0)
		{
			calcOpticalFlowPyrLK(pyramid1, pyramid2, pointVector1,
			 pointVector2, featuresFound, featuresError,
			 cvSize(pyrWindowSize,pyrWindowSize), 0,
			 cvTermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 0.2),
			 0,0.0001);
		}

		//store pyramid 2 in pyramid 1
		frame1 = frame2.clone();
		pyramid1.swap(pyramid2);

		//find the average displacement
		displacement = trajectoryCalc(pointVector1, pointVector2, featuresFound,
		 featuresError, KeyPointVector.size());
		//Compensate angle: must be done on RPI

		char xBuf[4]; char yBuf[4];
		int xBufLen = sprintf(xBuf, "%d", displacement.x);
		int yBufLen = sprintf(yBuf, "%d", displacement.y);
		comm->send(xBuf,xBufLen,ZMQ_NOBLOCK);
		comm->send(yBuf,yBufLen,ZMQ_NOBLOCK);

		opticalFlowVectors.push_back(displacement);
		mtx.lock();
	}
	t.join();

	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
	double fps = ((double)opticalFlowVectors.size())/time_span.count();

	ofstream myFile;
	myFile.open ("opticalFlow.txt");	

	myFile << "FPS: \t" << fps << endl;

	for (list<CvPoint>::iterator it=opticalFlowVectors.begin(); it!=opticalFlowVectors.end(); ++it){
		  myFile << "x:\t" << it->x << "\ty:\t" << it->y << endl;
	}

  	myFile.close();		

  	free(state_decompress);
}