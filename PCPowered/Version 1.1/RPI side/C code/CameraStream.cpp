#include "Communicator.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <zmq.h>

using namespace std;
using namespace cv;

int main(){
	Communicator* comm = new Communicator(512, "localhost", 9000, "*", 9001);

	//initialize camera
    VideoCapture cap(0);
    if(!cap.isOpened()){
        cout << "No camera found." << endl;
        return -1;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FPS, 30);

    //initialize frame
    Mat frame; Mat grayFrame;
    cap >> frame;
    
    //give information to PC about frame size
    char frameWidthBuf[3];
    char frameHeightBuf[3];
    sprintf(frameWidthBuf, "%d", frame.rows);
    sprintf(frameHeightBuf, "%d", frame.cols);
    comm->send(frameWidthBuf, 3, 0);
    comm->send(frameHeightBuf, 3, 0);
    int frameSize = frame.rows*frame.cols;

    //for encoding the frame
    vector<uchar> enc;

    while(1){
    	// capture gray image
        cap >> frame;
    	cvtColor(frame, grayFrame, CV_BGR2GRAY);

        // send encoded image
    	comm->send(grayFrame.data, frameSize, 0);
    }
}
