#!/usr/bin/python

import time
import zmq
import serial
import pickle
import numpy as np
from Queue import Empty

class serialModel():

    def __init__(self, dictionary = None, serialOutQueue=None):
        self.serialOutQueue = serialOutQueue
        self.inline = "default"
        self.outline = "default"
        self.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.dict = dictionary
        self.start = time.time()
        self.SLAM = ""
        self.dataNumber = 0
        self.dict['serialModelClosed'] = False
        self.dict['serialModelRunning'] = True
        self.run()

        while self.dict['serialModelRunning']:
            pass

    def __call__(self):
        self._quit()

    def run(self):
        self._initRecvSocket()
        self._initSendSocket()
        self._initSerial()
        while not self.dict['serialModelClosed']:
            self._readSerialData()
            self._readSocket() #receive optical flow from PC
            self._sendSocket() #send odometry to SLAM
            self._sendSerialData()
        print "out loop"
        self.dict['serialModelRunning'] = False
        self.dict['serialModelClosed'] = True

    ######################################################
    # To store dataset on flash memory                   #
    ######################################################
        #output = open('data.pkl','wb')
        #data = [self.timestamp[0:self.dataNumber],self.opticalX[0:self.dataNumber],self.opticalY[0:self.dataNumber]]
        #pickle.dump(data,output)
        #output.close()

        #print data

        self._quit()

    def _initRecvSocket(self):
        self.OFcontext = zmq.Context() # Set up client
        print "Connecting to OpticalFlow"
        self.OFsocket = self.OFcontext.socket(zmq.PAIR)
        self.OFsocket.bind("tcp://*:9002")
        print "Connected"

    def _initSendSocket(self):
        self.SLAMcontext = zmq.Context() # Set up client
        print "Connecting to SLAM"
        self.SLAMsocket = self.SLAMcontext.socket(zmq.PAIR)
        self.SLAMsocket.connect("tcp://localhost:8002") #RPI listens to data for SLAM
        print "Connected"

    def _initSerial(self):
        self.ser=serial.Serial("/dev/ttyAMA0", 115200)
        self.ser.flushInput()
        print "Serial initialized"
        
    ######################################################
    # Read incoming data from arduino                    #
    ######################################################
    def _readSerialData(self):
        if self.ser.inWaiting():
            self.inline = self.ser.readline().strip()

            if self.inline[:4] == "SLAM":
                #self.data = self.inline.split('|')
                #print self.data
                try:
                    self.SLAM = self.inline

                except IndexError:
                    print self.data
                    raise

    ######################################################
    # Read incoming data from opical flow                #
    ######################################################
    def _readSocket(self):
        self.xmov = self.OFsocket.recv()
        self.ymov = self.OFsocket.recv()
        self.timediff = self.OFsocket.recv()

        while self.xmov.endswith('\x00'):
            self.xmov = self.xmov[:-1]
        while self.ymov.endswith('\x00'):
            self.ymov = self.ymov[:-1]
        while self.timediff.endswith('\x00'):
            self.timediff = self.timediff[:-1]

        self.pred = '%.1f|%.1f|%d|m' %(float(self.xmov),float(self.ymov),int(self.timediff))
        #print self.pred
        self.ser.write(self.pred + "\n")

    ######################################################
    # Sending data to SLAM                               #
    ######################################################
    def _sendSocket(self):
        if(self.SLAM == ""):
            self.SLAMsocket.send(self.SLAM)

    ######################################################
    # Send predicted velocity over UART                  #
    ######################################################
    def _sendSerialData(self):
        while 1:
            try:
                self.outline = self.serialOutQueue.get(False)
                self.outline = str(self.outline)
            except Empty:
                break
            else:
                print "Line out " + self.outline
                self.ser.write(self.outline + "\n") # only write last given command

    def _quit(self):
        if self.dict['serialModelClosed']:
            return
        self.socket.close()
        print "serialModelFitter Closed"
