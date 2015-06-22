//in github 18/05/15 22:20


/*
  QuadControl
 Basic QuadCopter Control Program
 RPi             <->          Arduino
 QuadControl1.0.py             QuadControl1.0.c
 */

// Initialisation
void userhook_init()
{
  // Initialize the serial uart (Connection to RPI)
  // No need to initialize "Serial.begin()", this is done elsewhere
  Serial1.begin(115200);
  Serial1.flush();
  
  // Motor initialisation
  motors.set_update_rate(490);
  motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
  // Standard minimum and maximum settings
  motors.set_min_throttle(200);
  motors.set_max_throttle(800);
  motors.Init();  // initialise motors
 
  // Minimum values
  g.rc_1.radio_min = 1120; //1125 Roll
  g.rc_2.radio_min = 1120; //1123 Pitch
  g.rc_3.radio_min = 1120; //1128 Throttle
  g.rc_4.radio_min = 1120; //1126 Yaw
  // Maximum values
  g.rc_1.radio_max = 1922;
  g.rc_2.radio_max = 1921;
  g.rc_3.radio_max = 1922;
  g.rc_4.radio_max = 1922;
//  // Begin values
  g.rc_1.radio_in = 1499; //1540
  g.rc_2.radio_in = 1500; //1550
  g.rc_3.radio_in = 1125;
  g.rc_4.radio_in = 1533; //1520
  g.rc_5.radio_in = 1850;
  
  // Trim at neutral stick values
  g.rc_1.radio_trim = 1499;
  g.rc_2.radio_trim = 1500;
  g.rc_3.radio_trim = 1125;
  g.rc_4.radio_trim = 1533;
  
  // Control Mode initialisation
  control_mode = STABILIZE; // Set standard -> Stabilize mode

  //ik veronderstel dat deze waarde nodig is opdat aanpassingen in picth en roll een effect zouden hebben.
  g.rc_5.radio_in = 1300;
  
  //compenseer de drift of de yaw hoek
  STATE = GROUND;
}

// 100 Hz loop
void userhook_FastLoop()
{
 //haal accelero data en gyroscoop data
  accel = ins.get_accel();
  gyro = ins.get_gyro();
  
  // liftoff variable moet op true worden gezet wanneer de quad opstijgt --> middeltje om de hover throttle te vinden bij het opstijgen
  // neem het gemiddelde van de z-versnelling over 5 tijdsperioden
  totalAcceleroZ = totalAcceleroZ - rowAcceleroZ[indexAcceleroZ];
  rowAcceleroZ[indexAcceleroZ] = accel.z;
  totalAcceleroZ = totalAcceleroZ + rowAcceleroZ[indexAcceleroZ];
  
  indexAcceleroZ = indexAcceleroZ + 1;
  if(indexAcceleroZ >= 5)
  {
    indexAcceleroZ = 0;
  }
  avgAcceleroZ = totalAcceleroZ / 5.0;
  
  // Detecteer liftoff: wanneer is de zwaartekracht gecompenseerd?
  if (avgAcceleroZ < liftoffThreshold)
  {
    liftoff = true;
  }
  else{
    liftoff = false;
  }
  
}

// 20Hz loop
void userhook_MediumLoop()
{  
  // read in sonar altitude
  sonar_alt = read_sonar();
  if(STATE == ALTHOLD || STATE == POSHOLD || STATE == FLYFIGURE)
  {
    PIDAltitude();
    
    // Altitude correction: PWM value 1400, maximum 1800
    // voorkomt ook vliegen wanneer de batterij te plat is
    g.rc_3.set_pwm(max(min(throttleValue + correctionAltitude,1800),1400));
    
    //final
    //function generator for altitude
    if(((int)altitudeWayPoint) < altitudeDesired){
      altitudeWayPoint = altitudeWayPoint + altitudeStep;
    } else if(((int)altitudeWayPoint) > altitudeDesired){
      altitudeWayPoint = altitudeWayPoint - altitudeStep;
    }
    
    /*
    //voor een stapfunctie
    altitudeWayPoint = (float) altitudeDesired;
    */
    
    //throttleValue moet convergeren naar een waarde die zorgt dat de quadcopter hovered.
    //als de correctie constant blijft, maar de hoogte verandert niet, dan moet throttleValue aangepast worden.
    //experimenteel bepaald op 1515 --> enkel bij volledig opgeladen batterij
    
    //omdat de seriële verbinding slecht een beperkte snelheid heeft, is het best niet te veel informatie te zenden...
    /*
    if(!opticalFlow){
      Serial1.printf("alt:|%d|dalt:|%f|g.rc_3.radio_in:|%d|eradio_in:|%d\n", sonar_alt, altitudeWayPoint, g.rc_3.radio_in, correctionAltitude);
    }
    */
  }
}

void userhook_50Hz()
{
  // put your 50Hz code here
  // Wait for command from RPi
  while(Serial1.available()) 
  {
    c = Serial1.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    if(c == '\n')
    {
      stringComplete = true;
      break;
    }
    else
    {
      command = c;
    }
  }
  
  if(stringComplete)
  {
    stringComplete = false;
    // Execute command
    doCommand();
  }
  
  switch(STATE)
  {
    case GROUND:
      //do nothing
      break;
      
    case AUTOTAKEOFF:
      g.rc_3.set_pwm(throttleValue);
         
      if(liftoff){
        //initialiseer hoogte PID     
        newAltitudeError = 0;
        oldAltitudeError = 0;
        integralAltitudeError = 0.0; 
        derivativeAltitudeError = 0.0;
        correctionAltitude = 0;
           
        //stel een gewenste hoogte in
        altitudeWayPoint = (float)sonar_alt;
        altitudeDesired = 50; 
        
        STATE = ALTHOLD;       
      } else {
        throttleValue = throttleValue + 1;
      }
      
      break;
          
    case AUTOLAND:
      // Keep on lowering throttle
      g.rc_3.set_pwm(throttleValue);
    
      if(sonar_alt > 30){
        //do slow descend
        if(descender == 9){
          throttleValue = throttleValue - 1;
          descender = 0;
        } else {
          descender = descender + 1;
        }
        
        //1450 is normaal traag dalen
        throttleValue = max(throttleValue, 1450);
      } else {
        throttleValue = throttleValue - 1;
        throttleValue = max(throttleValue, 1120);
      }
            
      if(throttleValue < 1250)
      {
        STATE = GROUND;
        altitudeDesired = 0;
        g.rc_3.set_pwm(1120);
      }
      break;

    
    case ALTHOLD:      
      break;
    
    default:
      STATE = GROUND;
      break;
  }
}

