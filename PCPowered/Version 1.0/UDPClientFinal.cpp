#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <zmq.h>
#include <ctime>

using namespace std;
using namespace cv;

#define BUFLEN 512

int main(void){
    int n = 100;
    double duration;
    double total = 0.0;
    clock_t start;
    char buf[1];

    //initialize the sending socket
    int s = 0;
    void *send_context = zmq_ctx_new();
    void *send_socket = zmq_socket(send_context, ZMQ_PAIR);
    cout << "Initialize the sending socket" << endl;
    s = zmq_connect(send_socket, "tcp://192.168.2.1:9000");

    
    //initialize the receiving socket
    int r = 0;
    void *receive_context = zmq_ctx_new();
    void *receive_socket = zmq_socket(receive_context, ZMQ_PAIR);
    cout << "Initialize the receiving socket" << endl;
    r = zmq_bind(receive_socket, "tcp://*:9001");
    


    //initialize camera
    CvCapture* capture = cvCreateCameraCapture(1);
    if(!capture){
        cout<<"No camera found."<<endl;
        return 1;
    }

    int iter = 1;
    static IplImage *frame = NULL;

    while(iter<n){
        //start timer
        start = clock();

        //query frame            
        frame = cvQueryFrame(capture);
       	//cvShowImage("Sending", frame);
        //waitKey(0);
        IplImage *frame_gray = cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
        cvCvtColor(frame,frame_gray,CV_RGB2GRAY);

        //sending frame
        char* img = frame_gray->imageData;
        int IMG_SIZE = frame_gray->imageSize;

        for(int i=0; i<IMG_SIZE/BUFLEN+1; i++){
            s = zmq_send(send_socket, (img+i*BUFLEN), BUFLEN, 0);
            if (s == -1){
                    cout << "Error while sending" << endl;
                    return 1;
            }
        }
        
        //receive confirmation and time difference
        r = zmq_recv(receive_socket, buf, 1, 0);
        if(r!=-1){
            //stop timer
            duration = (clock()-start)/(double) CLOCKS_PER_SEC;
            //add to total
            total += duration;
        } else {
            cout << "Error receiving confirmation" << endl;
            cout << r << endl;
        }

        iter++;
    }

    zmq_close(send_socket);
    zmq_ctx_destroy(send_context);
    zmq_close(receive_socket);
    zmq_ctx_destroy(receive_context);

    cout << "Average time to send frame: " << total/n << endl;
    return 0;
}
