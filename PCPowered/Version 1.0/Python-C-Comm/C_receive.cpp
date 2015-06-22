#include <iostream>
#include <zmq.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include </usr/include/opencv2/opencv.hpp>


#define BUFSIZE 12

using namespace cv;
using namespace std;


int main(void){
	//initialize the sending socket
    int r = 0;
    void *recv_context = zmq_ctx_new();
    void *recv_socket = zmq_socket(recv_context, ZMQ_PAIR);
    r = zmq_bind(recv_socket, "tcp://*:9001");
    char buf[BUFSIZE];
    
    cout << "Waiting on sender" << endl;
    r = zmq_recv(recv_socket, buf, BUFSIZE, 0);

    cout << buf << endl;
    
    zmq_close(recv_socket);
    zmq_ctx_destroy(recv_context);
}