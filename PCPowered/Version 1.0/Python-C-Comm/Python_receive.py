import zmq
import random
import sys
import time


context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://*:9000")

msg = socket.recv()
print msg