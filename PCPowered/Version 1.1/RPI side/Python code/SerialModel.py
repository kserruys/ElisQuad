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
        self.alt = [None]*100000
        self.corx = [None]*100000
        self.posx = [None]*100000
        self.cory = [None]*100000
        self.posy = [None]*100000
        self.timestamp = [None]*100000
        self.dataNumber = 0
        self.dict['serialModelClosed'] = False
        self.dict['serialModelRunning'] = True
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
            self._sendSerialData()
        print "out loop"
        self.dict['serialModelRunning'] = False
        self.dict['serialModelClosed'] = True

    ######################################################
    # To store dataset on flash memory                   #
    ######################################################
        output = open('data.pkl','wb')
        data = [self.timestamp[0:self.dataNumber],self.alt[0:self.dataNumber],self.posx[0:self.dataNumber],self.corx[0:self.dataNumber],self.posy[0:self.dataNumber],self.cory[0:self.dataNumber]]
        pickle.dump(data,output)
        output.close()

        print data

        self._quit()

    def _initSocket(self):
        self.context = zmq.Context() # Set up client
        print "Connecting to OpticalFlow"
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.bind("tcp://*:9002")
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

            if self.inline[:4] != "alt:" and self.inline[:7] != "OPTPOS:":
                print self.inline

            if self.inline[:4] == "corX":
                self.data = self.inline.split('|')
                #print self.data
                try:
                    self.alt[self.dataNumber] = self.data[9]
                    self.corx[self.dataNumber] = self.data[1]
                    self.posx[self.dataNumber] = self.data[3]
                    self.cory[self.dataNumber] = self.data[5]
                    self.posy[self.dataNumber] = self.data[7]
                    self.timestamp[self.dataNumber] = time.time()
                    self.dataNumber += 1

                except IndexError:
                    print self.dataNumber
                    print self.data
                    raise

    ######################################################
    # Read incoming data from opical flow                #
    ######################################################
    def _readSocket(self):
        self.xmov = self.socket.recv()
        self.ymov = self.socket.recv()
        self.timediff = self.socket.recv()

        while self.xmov.endswith('\x00'):
            self.xmov = self.xmov[:-1]
        while self.ymov.endswith('\x00'):
            self.ymov = self.ymov[:-1]
        while self.timediff.endswith('\x00'):
            self.timediff = self.timediff[:-1]

        self.pred = '%.2f|%.2f|%d|m' %(float(self.xmov),float(self.ymov),int(self.timediff))
        #print self.pred
        self.ser.write(self.pred + "\n")

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
