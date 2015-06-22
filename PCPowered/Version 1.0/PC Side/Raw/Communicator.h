#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <cstring>

#include <zmq.h>

using namespace std;

class Communicator
{
	public:
		Communicator(int buf_size, string send_IP, int send_port, string recv_IP, int recv_port);
		~Communicator();

		int send(void* msg, int msg_size, int mode);
		int recv(void* msg, int msg_size, int mode);

	private:
		int buf_size;

		int recv_port;
		int send_port;

		string send_IP;
		string recv_IP;

		void* recv_context;
		void* send_context;

		void* recv_socket;
		void* send_socket;

		int init_sender();
		int init_recver();

};

Communicator::Communicator(int buf_size, string send_IP, int send_port, string recv_IP, int recv_port){
	this->buf_size = buf_size;
	this->send_IP = send_IP;
	this->recv_IP = recv_IP;
	this->recv_port = recv_port;
	this->send_port = send_port;

	init_recver();
	init_sender();
}

Communicator::~Communicator(){
	zmq_close(send_socket);
	zmq_ctx_destroy(send_context);
	
	zmq_close(recv_socket);
	zmq_ctx_destroy(recv_context);
	
}


/********INITIALISATION*********/
int Communicator::init_recver(){
	recv_context = zmq_ctx_new();
	recv_socket = zmq_socket(recv_context, ZMQ_PAIR);
	
	string address = "tcp://" + recv_IP	+ ":" + to_string(recv_port);

	return zmq_bind(recv_socket, address.c_str());
}

int Communicator::init_sender(){
	send_context = zmq_ctx_new();
	send_socket = zmq_socket(send_context, ZMQ_PAIR);
	
	string address = "tcp://" + send_IP	+ ":" + to_string(send_port);

	return zmq_connect(send_socket, address.c_str());
}

/********SENDING*********/
int Communicator::send(void* msg, int msg_size, int mode){
	int s;
	char buf[buf_size];
	int i=0;

	// first blocks have length buf_size
	for(i=0; i<msg_size/buf_size; i++){
		memcpy(buf, msg+i*buf_size, buf_size);

		s = zmq_send(send_socket, buf, buf_size, mode);
		if(s < 0){
			cout << "error sending message." << endl;
			return -1;
		}
	}
	// last block may have a different length
	int last_block_size = msg_size-buf_size*i;
	char buf2[last_block_size];

	memcpy(buf2, msg+i*buf_size, last_block_size);

	s = zmq_send(send_socket, buf2, last_block_size, mode);
	if(s < 0){
		cout << "error sending message." << endl;
		return -1;
	}
}

/********RECEIVING*********/
int Communicator::recv(void* msg, int msg_size, int mode){
	int r;
	char buf[buf_size];
	int i=0;

	// first few blocks have size buf_size
	for(i=0; i<msg_size/buf_size; i++){
		r = zmq_recv(recv_socket, buf, buf_size, mode);
		if(r < 0){
			cout << "error receiving message." << endl;
			return -1;
		}

		memcpy(msg+i*buf_size, buf, buf_size);
	}

	// last block can have a different size
	int last_block_size = msg_size-buf_size*i;
	char buf2[last_block_size];
	r = zmq_recv(recv_socket, buf2, last_block_size, mode);
	if(r < 0){
		cout << "error receiving message." << endl;
		return -1;
	}

	memcpy(msg+i*buf_size, buf2, last_block_size);

	return 0;
}