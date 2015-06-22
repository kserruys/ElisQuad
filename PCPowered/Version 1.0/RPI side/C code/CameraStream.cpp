#include "Communicator.h"
#include "quicklz.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <zmq.h>
#include <ctime>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;

int main(){
	Communicator* comm = new Communicator(512, "192.168.2.1", 9000, "*", 9001);

	//initialize camera
    VideoCapture cap(0);
    if(!cap.isOpened()){
        cout << "No camera found." << endl;
        return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
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
    size_t frameSize = frame.rows*frame.cols;

    //for encoding the frame
    char* enc = new char[frameSize];
    qlz_state_compress *state_compress = (qlz_state_compress *)malloc(sizeof(qlz_state_compress));

    while(1){
    	// capture gray image
        cap >> frame;
    	cvtColor(frame, grayFrame, CV_BGR2GRAY);

        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        //do lz4 encoding
        size_t encSize = qlz_compress((const void*) grayFrame.data, enc, frameSize, state_compress);

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

        cout << time_span.count() << endl;

        //get size encoded frame (use standard message length of 6)
        char encSizeBuf[6];
        if(encSize<100000)
            sprintf(encSizeBuf, "0%d", encSize);
        else
            sprintf(encSizeBuf, "%d", encSize);

        //send size encoded frame
        comm->send(encSizeBuf, 6, 0);

        // send encoded image
    	comm->send(enc, encSize, 0);
    }
}