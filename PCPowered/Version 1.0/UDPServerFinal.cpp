#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include </usr/include/opencv2/opencv.hpp>
#include <zmq.h>

#define IMG_SIZE 307200
#define BUFLEN 512
    
using namespace cv;
using namespace std;
   
int main(void){
	char* img = (char*) malloc(IMG_SIZE);
	char buf[BUFLEN];

	//init receiving socket	
	int r;
	void *receive_context = zmq_ctx_new ();
	void *receive_socket = zmq_socket(receive_context, ZMQ_PAIR);
	cout << "Initialize the receiving socket" << endl;
	r = zmq_bind(receive_socket, "tcp://*:9000");

	
    //initialize the sending socket
    int s;
    void *send_context = zmq_ctx_new ();
    void *send_socket = zmq_socket(send_context, ZMQ_PAIR);
    cout << "Initialize the sending socket" << endl;
    s = zmq_connect(send_socket, "tcp://10.42.0.13:9001");
	

	while(1){
		//receive chunks of data
		for(int i=0; i<IMG_SIZE/BUFLEN+1; i++){
			r = zmq_recv(receive_socket, buf, BUFLEN, 0);

	 		if (r == -1){
				cout << "Error receiving image." << endl;
				return 1;
			}
			
			memcpy(img+i*BUFLEN, buf, BUFLEN);
		}
		
		//convert data to a frame
		IplImage* frame= cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
		memcpy(frame->imageData, img, IMG_SIZE);

		
		//show frame
		cvNamedWindow("Received", CV_WINDOW_AUTOSIZE);
		cvShowImage("Received", frame);
		if(waitKey(10) == 27) break;
		
		
		char confirmation[1];
		confirmation[0] = '1';

		//send a confirmation: use second socket
		s = zmq_send(send_socket, confirmation, 1, ZMQ_NOBLOCK);
	}    

    zmq_close(send_socket);
   	zmq_ctx_destroy(send_context);
    zmq_close(receive_socket);
    zmq_ctx_destroy(receive_context);

    return 0;
}
