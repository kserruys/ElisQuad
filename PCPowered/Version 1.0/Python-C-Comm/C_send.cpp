#include <iostream>
#include <zmq.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


using namespace std;

int main(void){
	//initialize the sending socket
    int s = 0;
    void *send_context = zmq_ctx_new();
    void *send_socket = zmq_socket(send_context, ZMQ_PAIR);
    s = zmq_connect(send_socket, "tcp://192.168.2.3:9002");

    s = zmq_send(send_socket, "2", 1, ZMQ_NOBLOCK);
    s = zmq_send(send_socket, "-4", 2, ZMQ_NOBLOCK);

    zmq_close(send_socket);
    zmq_ctx_destroy(send_context);
}