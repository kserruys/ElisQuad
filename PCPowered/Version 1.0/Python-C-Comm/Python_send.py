import zmq
import random
import sys
import time
import cv2


context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.connect("tcp://192.168.2.1:9000")

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

socket.send(frame.imageData)

