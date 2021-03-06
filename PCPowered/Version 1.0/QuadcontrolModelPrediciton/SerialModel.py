#!/usr/bin/python

import time
import zmq
import serial
import pickle
import numpy as np
from Queue import Empty
from sklearn.gaussian_process import GaussianProcess
from sklearn.externals import joblib

class serialModel():

    def __init__(self, dictionary = None, serialOutQueue=None):
        self.serialOutQueue = serialOutQueue
	self.inline = "default"
	self.outline = "default"
        self.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        #self.xy = np.atleast_2d([[0.0]*9 for i in range(0,2)])
        self.dict = dictionary
        self.start = time.time()
        self.opticalX = [None]*10000
        self.opticalY = [None]*10000
        self.accelX = [None]*10000
        self.accelY = [None]*10000
        self.gyroX = [None]*10000
        self.gyroY = [None]*10000
        self.timestamp = [None]*10000
        self.dataNumber = 0
        self.dict['serialModelClosed'] = False
        self.dict['serialModelRunning'] = True
        print "Model loading..."
        self.model = joblib.load('BigModel3T.pkl') 
        print "Done!"
        self.run()
        while self.dict['serialModelRunning']:
            pass

    def __call__(self):
        self._quit()

    def run(self):
        self._initSocket()
        self._initSerial()
	while not self.dict['serialModelClosed']:
	    self._readSerialData()
            self._readSocket()
            self._modelFit()
	    self._sendSerialData()
        print "out loop"
        self.dict['serialModelRunning'] = False
        self.dict['serialModelClosed'] = True

	######################################################
	# To store dataset on flash memory                   #
	######################################################
        output = open('data.pkl','wb')
        data = [self.opticalX[0:self.dataNumber],self.opticalY[0:self.dataNumber],self.accelX[0:self.dataNumber],self.accelY[0:self.dataNumber],self.gyroX[0:self.dataNumber],self.gyroY[0:self.dataNumber],self.timestamp[0:self.dataNumber]]
        pickle.dump(data,output)
        output.close()

        print data

        self._quit()

    def _initSocket(self):
        self.context = zmq.Context() # Set up client
        print "Connecting to OpticalFlow"
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.connect("tcp://localhost:5555")
        print "Connected"

    def _initSerial(self):
        self.ser=serial.Serial("/dev/ttyAMA0", 115200)
        self.ser.flushInput()
        print "Serial initialized"
        
    	######################################################
	# Read incoming data from arduino                    #
	######################################################
    def _readSerialData(self):
        while self.ser.inWaiting():
            self.inline = self.ser.readline().strip()
            if self.inline[:4] != "GYR:":
                print self.inline
        if self.inline[:4] == "GYR:":
            self.data = self.inline.split('|')
            #print self.data
            self.opticalX[self.dataNumber] = self.xmov
            self.opticalY[self.dataNumber] = self.ymov
            self.accelX[self.dataNumber] = self.data[1]
            self.accelY[self.dataNumber] = self.data[2]
            self.gyroX[self.dataNumber] = self.data[4]
            self.gyroY[self.dataNumber] = self.data[5]
            self.timestamp[self.dataNumber] = time.time()
            self.dataNumber += 1

	    # Normalisation
            self.xy[0,7] = float(self.data[1])/0.8
            self.xy[0,8] = float(self.data[4])/4.0
            self.xy[1,7] = float(self.data[2])/0.8
            self.xy[1,8] = float(self.data[5])/4.0

    	######################################################
	# Read incoming data from opical flow                #
	######################################################
    def _readSocket(self):
	self.xmov = self.socket.recv()
	self.ymov = self.socket.recv()
	while self.xmov.endswith('\x00'):
            self.xmov = self.xmov[:-1]
        while self.ymov.endswith('\x00'):
            self.ymov = self.ymov[:-1]
        #print "x: %f,y: %f" %(float(self.xmov),float(self.ymov))
        self.xy[:,0:6] = self.xy[:,3:]

    	######################################################
	# Predict velocity using data                        #
	######################################################
    def _modelFit(self):
        pred = self.model.predict(self.xy) # fit model
        #print self.xy
	#calcTime = time.time() - start
        #print "prediction: x:%f, y:%f, time taken: %f" %(pred[0],pred[1],calcTime)
        self.pred = '%d|%d||m' %(int(self.xmov),int(self.ymov))
        #self.pred = '%d|%d|%d|%d||m' %(int(pred[0]),int(pred[1]),int(self.xmov),int(self.ymov))
        #self.pred = '%.2f|%.2f||m' %(float(self.xmov)*25.0,float(self.ymov)*25.0)
        self.ser.write(self.pred + "\n")

    	######################################################
	# Send predicted velocity over UART                  #
	######################################################
    def _sendSerialData(self):
        #print self.pred
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
