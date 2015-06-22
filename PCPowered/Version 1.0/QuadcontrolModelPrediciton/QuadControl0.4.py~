#!/usr/bin/python

from multiprocessing import Process, Manager, Queue
import time
import sys
from Keyboard import keyboardReader
from SerialModel import serialModel

if __name__ == '__main__':
    manager = Manager()
    serialOutQueue = Queue()
    d = manager.dict()

    print "QuadCopter Control 0.4\nBasic serial link between Arduino and RPi"
    print "Client socket model implemented, for communication between python and C\n"
    print "Type in 'h' for help\n"

    serialModel = Process(target = serialModel, args=(d, serialOutQueue))
    serialModel.start()

    keyboardReader = Process(target = keyboardReader(sys.stdin, d, serialOutQueue))
    keyboardReader.start()

    keyboardReader.join()
    serialModel.join()

    print "program closed"
