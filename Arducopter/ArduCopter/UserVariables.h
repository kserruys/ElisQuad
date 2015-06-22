// agmatthews USERHOOKS
// user defined variables

#define GROUND  	0
#define TAKEOFF  	1
#define AUTOTAKEOFF	2
#define FLYING  	3
#define ALTHOLD		4
#define POSHOLD 	5
#define FLYFIGURE    	6
#define LANDING    	7
#define AUTOLAND	8

//************************FLYING VARIABLES*****************************//
Vector3f gyro, accel;
int throttleValue = 0, rollValue = 0, pitchValue = 0;
int modelVelocityX = 0, modelVelocityY = 0;
boolean opticalFlow = false;
unsigned char STATE = GROUND;

// for yawDiff calculation
long newYaw = 0, oldYaw = 0;
int yawDiff = 0;
int yawDiffT0 = 0,yawDiffT1 = 0,yawDiffT2 = 0,yawDiffT3 = 0,yawDiffT4 = 0;

//************************LIFTOFF****************************//
boolean liftoff = false;
float liftoffThreshold = -10.9;

//no more used
float rowAcceleroZ[5] = {0.0};
int indexAcceleroZ = 0;
float totalAcceleroZ = 0.0;
float avgAcceleroZ = -9.8; //ongeveer de zwaartekracht



//********************COMMUNICATION VARIABLES*************************//
boolean stringComplete = false;
String readString;
char command, c;
char charBuf[10] = {0};
int antiCongest = 0;
int tempIndex = 0, index = 0;

//******************OPTICAL FLOW VARIABLES***********************//
//receiving optical flow data
float opticalDisplacementX = 0.0, opticalDisplacementY = 0.0;
int timeDiff = 0;

//for calculating real movement
float displacementX = 0.0, displacementY = 0.0;
float positionX = 0.0, positionY = 0.0;
float velocityX = 0.0, velocityY = 0.0;

// desired
float desiredVelocityX = 0.0, desiredVelocityY = 0.0;

//for roll and pitch compensation of optical flow
float compOpticalDisplacementX = 0.0, compOpticalDisplacementY = 0.0;
long newRoll = 0, oldRoll = 0;
long newPitch = 0, oldPitch = 0;
float rollDiff = 0.0, rollDiffT0 = 0.0, rollDiffT1 = 0.0, rollDiffT2 = 0.0, rollDiffT3 = 0.0, rollDiffT4 = 0.0;
float pitchDiff = 0.0, pitchDiffT0 = 0.0, pitchDiffT1 = 0.0, pitchDiffT2 = 0.0, pitchDiffT3 = 0.0, pitchDiffT4 = 0.0;

//*******************ALTITUDE PID VARIABLES*************************//
int newAltitudeError = 0, oldAltitudeError = 0;
float derivativeAltitudeError = 0.0, integralAltitudeError = 0.0;
float altitudeWayPoint = 0.0, altitudeStep = 0.5;
int correctionAltitude = 0;
float pAltitude = 1.5, iAltitude = 0.1, dAltitude = 0.8;
int altitudeDesired = 0, descender = 0;

//******************VELOCITY PI VARIABLES***************************//
float newVelocityErrorX = 0.0, newVelocityErrorY = 0.0;
float integralVelocityErrorX = 0.0, integralVelocityErrorY = 0.0;
int correctionX = 0, correctionY = 0;
// dOptical wordt niet meer gebruikt
float pOptical=10.0, iOptical=0.0, dOptical=0.0;
