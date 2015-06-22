//============================================================================
// Name        : OpticalFlow.cpp
// Author      : Wouter Van der Gucht
// Version     : 0.1
// Description : Tracking of webcam(!) movements, using opticalFlow
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <time.h>

#define BILLION 1E9

using namespace std;
using namespace cv;

char c;
int corner_count = 2000;
int win_size = 10, FASTThreshold = 40;
int mov_x_total = 0, mov_y_total = 0, good_points = 0;
float sec, fps, inlezen, goodfeatures, opticalflow, trajectory;
struct timespec requestStart, requestTemp, requestEnd;
CvPoint actualPoint = cvPoint(100,100); // Relative starting point of tracking
CvPoint nextPoint;
CvPoint2D32f mov_vecs[2000];
vector<KeyPoint> KeyPointVector;
vector<Mat> prevPyramid, nextPyramid;
vector<uchar> features_found;
vector<float> features_error;
vector<Point2f> PointVectorA, PointVectorB;

// Function allocateOnDemand
// DONE: allocates memory when needed, else uses the memory that is already allocated
static void allocateOnDemand( IplImage **image, CvSize img_sz, int depth, int channels )
{
	if ( *image != NULL ) return;

	*image = cvCreateImage(img_sz, depth, channels);
	if ( *image == NULL )
	{
		printf("Out of memory!\n");
		exit(1);
	}
}

// Function trajectoryCalc
// DONE => Just averaging out all the corner vectors!
// PLANNING => Don't take into account vectors that differ a lot from the average
static CvPoint trajectoryCalc(vector<Point2f> PointVectorA,vector<Point2f> PointVectorB, vector<uchar> features_found, vector<float> features_error, int feature_number, CvPoint actualPoint)
{

	for(int i=0; i < feature_number; i++)
	{
		if(features_found[i]==0 || features_error[i]>20)
		{
			continue;
		}
		mov_vecs[i].x= cvRound(PointVectorA[i].x) - cvRound(PointVectorB[i].x);
		mov_x_total += mov_vecs[i].x;
		mov_vecs[i].y= cvRound(PointVectorA[i].y) - cvRound(PointVectorB[i].y);
		mov_y_total += mov_vecs[i].y;
		good_points++;
	}

	if(good_points != 0)
	{
		nextPoint = cvPoint(actualPoint.x + round(mov_x_total/good_points),actualPoint.y + round(mov_y_total/good_points));
	}
	else
	{
		nextPoint = actualPoint;
	}
	mov_x_total = 0;
	mov_y_total = 0;
	good_points = 0;

	return nextPoint;
}

// Function compensateAngle
// DONE: Read in sensor values from APM (altitude, gyroscope(x;y))
static CvPoint compensateAngle(CvPoint actualPoint,CvPoint nextPoint)
{
	return nextPoint;
}

int main(int argc, char** argv)
{
	//cvNamedWindow("Frame1", CV_WINDOW_AUTOSIZE);
	//cvMoveWindow("Frame1", 100, 100);
	//cvNamedWindow("Frame2", CV_WINDOW_AUTOSIZE);
	//cvMoveWindow("Frame2", 800, 100);
	cvNamedWindow("LKpyr_optical_flow", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("LKpyr_optical_flow", 100, 600);
	cvNamedWindow("Trajectory_Calc", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Trajectory_Calc", 800, 600);
	CvCapture *g_capture = cvCreateCameraCapture(0); // Read out display from webcam

	if( !g_capture )
	{
		printf("Error when reading steam");
	}

	cvSetCaptureProperty( g_capture, CV_CAP_PROP_FRAME_WIDTH, 320 );// 920
	cvSetCaptureProperty( g_capture, CV_CAP_PROP_FRAME_HEIGHT, 240 );// 720

	CvSize img_sz = cvGetSize(cvQueryFrame(g_capture));

	static IplImage *frame1 = NULL, *frame2 = NULL, *frame3 = NULL, *frame4 = NULL, *frame1_gray = NULL, *frame2_gray = NULL;


	frame1 = cvQueryFrame(g_capture);
	allocateOnDemand( &frame1_gray, img_sz, IPL_DEPTH_8U, 1 );
	cvConvertImage(frame1, frame1_gray);
	//cvShowImage("Frame1", frame1);

	Mat FirstFrame(frame1_gray);
	cv::buildOpticalFlowPyramid(FirstFrame, prevPyramid, cvSize(win_size,win_size), 0);

	int counter=0;
	printf("start timing\n");
	clock_gettime(CLOCK_REALTIME, &requestStart);

	while(counter<100)
	{

	clock_gettime(CLOCK_REALTIME, &requestTemp);


		// Take 2 images directly from our webcam stream
		// IMPORTANT: WebCam buffers 1(?) frame, so we have to take this into account
		// Frame1 will be the frame taken (cvWaitKey(time);) time ago. Frame 2 will
		// be the frame taken at the cvQuery of frame1
		try
		{

		frame2 = cvQueryFrame(g_capture);
		if(!frame2) break;
		allocateOnDemand( &frame2_gray, img_sz, IPL_DEPTH_8U, 1 );
		cvConvertImage(frame2, frame2_gray);
		//cvShowImage("Frame2", frame2);
		}
		catch(...)
		{
			continue;
		}
		allocateOnDemand( &frame3, img_sz, IPL_DEPTH_8U, 3 );
		cvConvertImage(frame2, frame3);

		allocateOnDemand( &frame4, img_sz, IPL_DEPTH_8U, 3 );
		cvConvertImage(frame2, frame4);

	clock_gettime(CLOCK_REALTIME, &requestEnd);
	//time between 2 frames
	inlezen = (requestEnd.tv_sec - requestTemp.tv_sec) + (requestEnd.tv_nsec - requestTemp.tv_nsec)/BILLION;

		// Get The features we want to track
		Mat FrameA(frame1_gray);
		//KeyPointVector.size()/corner_count

		FeatureDetector * detector = new FastFeatureDetector(FASTThreshold,true);
//		FeatureDetector * detector = new DenseFeatureDetector();
//		FeatureDetector * detector = new GFTTDetector();
//		FeatureDetector * detector = new MSER();
//		FeatureDetector * detector = new ORB(corner_count,1.2,8,20,0,2,ORB::FAST_SCORE,20);
		detector->detect(FrameA, KeyPointVector);
		delete detector;

		if(KeyPointVector.size() > 30){FASTThreshold++;}
		else{FASTThreshold--;}

	clock_gettime(CLOCK_REALTIME, &requestEnd);
	//Time to find features
	goodfeatures = (requestEnd.tv_sec - requestTemp.tv_sec) + (requestEnd.tv_nsec - requestTemp.tv_nsec)/BILLION;

		Mat FrameB(frame2_gray);

		cv::buildOpticalFlowPyramid(FrameB, nextPyramid, cvSize(win_size,win_size), 0);
		cv::KeyPoint::convert(KeyPointVector, PointVectorA);

		if(KeyPointVector.size() > 0)
		{
			calcOpticalFlowPyrLK(	prevPyramid,
						nextPyramid,
						PointVectorA,
						PointVectorB,
						features_found,
						features_error,
						cvSize(win_size,win_size),
						0,
						cvTermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 0.2),
						0,
						0.0001);
		}
		prevPyramid.swap(nextPyramid);

	//Time for opticalflow
	clock_gettime(CLOCK_REALTIME, &requestEnd);
	opticalflow = (requestEnd.tv_sec - requestTemp.tv_sec) + (requestEnd.tv_nsec - requestTemp.tv_nsec)/BILLION;

		// Make an image of our flow vectors
		for(int i=0; i < (signed)KeyPointVector.size(); i++)
		{
			if(features_found[i]==0 || features_error[i]>20)
			{
				//printf("Error is: %f\n",featur_error[i]);
				continue;
			}
			//printf("Got it\n");
			CvPoint p0 = cvPoint(
					cvRound(PointVectorA[i].x),
					cvRound(PointVectorA[i].y));

			CvPoint p1 = cvPoint(
					cvRound(PointVectorB[i].x),
					cvRound(PointVectorB[i].y));

			cvLine(frame3, p0, p1, CV_RGB(255,0,0),2);
		}

		//Find average displacement
		nextPoint = trajectoryCalc(PointVectorA,PointVectorB,features_found,features_error,KeyPointVector.size(),actualPoint);
		nextPoint = compensateAngle(actualPoint, nextPoint);
		//Draw displacement on frame4.
		cvLine(frame4, actualPoint, nextPoint, CV_RGB(255,0,0),2);
		actualPoint = nextPoint;

	//Time for finding average displacement (trajectory)
	clock_gettime(CLOCK_REALTIME, &requestEnd);
	trajectory = (requestEnd.tv_sec - requestTemp.tv_sec) + (requestEnd.tv_nsec - requestTemp.tv_nsec)/BILLION;

		cvShowImage("LKpyr_optical_flow", frame3);
		cvShowImage("Trajectory_Calc", frame4);

		c = cvWaitKey(1);
		if(c==27) break;

		frame1 = frame2;
		allocateOnDemand( &frame1_gray, img_sz, IPL_DEPTH_8U, 1 );
		cvConvertImage(frame1, frame1_gray);
		//cvShowImage("Frame1", frame1);

		++counter;
		printf("%d", counter);

	}

	clock_gettime(CLOCK_REALTIME, &requestEnd);
        sec = (requestEnd.tv_sec - requestStart.tv_sec) + (requestEnd.tv_nsec - requestStart.tv_nsec)/BILLION;
	fps = counter/sec;

	printf("sec:%lf, fps:%lf\n",sec, fps);
	printf("size KeyPointVector: %d\n",KeyPointVector.size());
	printf("Last run: inlezen=%lf, goodfeatures=%lf, opticalflow=%lf, trajectory=%lf\n",inlezen,goodfeatures-inlezen,opticalflow-goodfeatures,trajectory-opticalflow);

	cvReleaseCapture(&g_capture);
	cvDestroyWindow("Frame1");
	cvDestroyWindow("Frame2");
	cvDestroyWindow("LKpyr_optical_flow");
	cvDestroyWindow("Trajectory_Calc");
}
