#! /usr/bin/python

import time

class keyboardReader():

    def __init__(self, stdin = None, dictionary = None, serialOutQueue = None):
        self.line = "default"
        self.serialOutQueue = serialOutQueue
        self.dict = dictionary
        self.stdin = stdin
        self.dict['keyboardClosed'] = False
        self.dict['keyboardRunning'] = True

    	self.commands={ "z" :self.forwards,
    			"s" :self.backwards,
    			"q" :self.left,
    			"d" :self.right,
    			"a" :self.up,
    			"e" :self.down,
    			"h" :self.help,
    			"f" :self.variables,
    			"r" :self.arm,
    			"t" :self.disarm, 
    			"c" :self.althold,
    			"v" :self.altrelease,
                "pidvel" :self.velpidset,
                "pidpos" :self.pospidset,
                "pidsonar" :self.sonarpidset,
    			"x" :self.exit	}
                
	print "Keyboard init"
        self.run()
        while self.dict['keyboardRunning']:
            pass

    def __call__(self):
        self._quit()

    def run(self):
        while not self.dict['keyboardClosed']:
            try:
                self._readKeyBoard()
            except KeyboardInterrupt:
                pass
        self.dict['keyboardRunning'] = False
        self.dict['keyboardClosed'] = True
        self._quit()

    def _readKeyBoard(self):
        print "Keyboard running"
        print "Enter command -->"
        self.line = self.stdin.readline().strip()
	if(self.line in self.commands):
	    self.commands[self.line]()
	    self._queueSerialData()
	else:
	    print 'Error in input!'

    def _queueSerialData(self):
        self.serialOutQueue.put(self.line)

    def _quit(self):
        if self.dict['keyboardClosed']:
            return 
        print "keyboardReader Closed"

    # Code more reader-friendly by using definitions
    def forwards(self):
	   print "Forward Command\n"
    def backwards(self):
	   print "Backward Command\n"
    def left(self):
	   print "Left Command\n"
    def right(self):
	   print "Right Command\n"
    def up(self):
	   print "Up Command\n"
    def down(self):
	   print "Down Command\n"
    def help(self):
    	print "Help Function"
    	print "---------------"
    	print "z -> Forwards\ns -> Backwards"
    	print "q -> Left\nd -> Right"
    	print "w -> Spin Left\nx -> Spin Right"
    	print "c -> Altitude Hold\nv -> Altidude Release"
    	print "a -> Up\ne -> Down\nx -> Exit Program\n"
    def variables(self):
        print "Print variables"
    def althold(self):
        print "Holding Altitude"
    def altrelease(self):
        print "Manual Throttle"
    def arm(self):
        print "Arming motors"
    def disarm(self):
        print "Disarming motors"
    def velpidset(self):
        print "Input new P value"
        self.pOptical = self.stdin.readline().strip()
        print "Input new D value"
        self.dOptical = self.stdin.readline().strip()
        print "Input new I value"
        self.iOptical = self.stdin.readline().strip()
        self.line = self.pOptical + "|" + self.dOptical + "|" + self.iOptical + "||l"
    def pospidset(self):
        print "Input new P value"
        self.pOptical = self.stdin.readline().strip()
        print "Input new D value"
        self.dOptical = self.stdin.readline().strip()
        print "Input new I value"
        self.iOptical = self.stdin.readline().strip()
        self.line = self.pOptical + "|" + self.dOptical + "|" + self.iOptical + "||o"
    def sonarpidset(self):
        print "Input new P value"
        self.pSonar = self.stdin.readline().strip()
        print "Input new D value"
        self.dSonar = self.stdin.readline().strip()
        print "Input new I value"
        self.iSonar = self.stdin.readline().strip()
        self.line = self.pSonar + "|" + self.dSonar + "|" + self.iSonar + "||k"
    def exit(self):
        print "Closing QuadControl\n"
        self.dict['keyboardRunning'] = False
        self.dict['keyboardClosed'] = True
        self.dict['serialModelRunning'] = False
        self.dict['serialModelClosed'] = True

