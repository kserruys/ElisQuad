void doCommand()
{
  switch(command)
  {
  case 'z':
    Serial.print("Going Forward\n");
    Serial1.print("Going Forward\n");
    g.rc_2.radio_in = g.rc_2.radio_in + 50;
    g.rc_2.set_pwm(g.rc_2.radio_in);
    break;
  case 's':
    Serial.print("Going Backward\n");
    Serial1.print("Going Backward\n");
    g.rc_2.radio_in = g.rc_2.radio_in - 50;
    g.rc_2.set_pwm(g.rc_2.radio_in);
    break;
  case 'q':
    Serial.print("Rolling Left\n");
    Serial1.print("Rolling Left\n");
    g.rc_1.radio_in = g.rc_1.radio_in - 50;
    g.rc_1.set_pwm(g.rc_1.radio_in);
    break;
  case 'd':
    Serial.print("Rolling Right\n");
    Serial1.print("Rolling Right\n");
    g.rc_1.radio_in = g.rc_1.radio_in + 50;
    g.rc_1.set_pwm(g.rc_1.radio_in);
    break;
  case 'a':
    Serial.print("More Throttle\n");
    Serial1.print("More Throttle\n");
    if(STATE == ALTHOLD | STATE == POSHOLD)
    {
      altitudeDesired = altitudeDesired + 5;
    }
    else
    {
      g.rc_3.radio_in = g.rc_3.radio_in + 10;
      g.rc_3.set_pwm(g.rc_3.radio_in);
    }
    break;
  case 'e':
    Serial.print("Less Throttle\n");
    Serial1.print("Less Throttle\n");
    if(STATE == ALTHOLD | STATE == POSHOLD)
    {
      if(altitudeDesired > 5){
        altitudeDesired = altitudeDesired - 5;
      }
    }
    else
    {
      g.rc_3.radio_in = g.rc_3.radio_in - 10;
      g.rc_3.set_pwm(g.rc_3.radio_in);
    }
    break;
  case 'c':
    if(STATE == FLYING)
    {
      Serial.print("Altitude Hold\n");
      Serial1.print("Altitude Hold\n");
      g.rc_5.radio_in = 1600;
      throttleValue = g.rc_3.radio_in;
      force_new_altitude(min(sonar_alt, 200));
    }
    else
    {
      Serial.print("Position Hold\n");
      Serial1.print("Position Hold\n");
      g.rc_5.radio_in = 1300;
      rollValue = g.rc_1.radio_in;
      pitchValue = g.rc_2.radio_in;
    }
    break;
    
  case 'v':
    Serial.print("Manual Control\n");
    Serial1.print("Manual Control\n");
    STATE = FLYING;
    g.rc_5.radio_in = 1800;
    break;
    
  case 'f':
    // Display some general variables
    Serial1.print("Throttle Values\n");
    Serial1.printf("RC in: %d\n",g.rc_3.radio_in);
    Serial1.printf("Control in: %d\n",g.rc_3.control_in);
    Serial1.printf("PWM out: %d\n",g.rc_3.pwm_out);
    Serial1.printf("Servo out: %d\n",g.rc_3.servo_out);
    Serial1.print("------------------\n");
    Serial1.print("Rate Values\n");
    Serial1.printf("Roll rate Kp: %f\n",g.pid_rate_roll.kP());
    Serial1.printf("Pitch rate Kp: %f\n",g.pid_rate_pitch.kP());
    Serial1.printf("Yaw rate Kp: %f\n",g.pid_rate_yaw.kP());
    Serial1.printf("Stabilize roll rate Kp: %f\n",g.pi_stabilize_roll.kP());
    Serial1.printf("Stabilize pitch rate Kp: %f\n",g.pi_stabilize_pitch.kP());
    Serial1.printf("Stabilize yaw rate Kp: %f\n",g.pi_stabilize_yaw.kP());
    Serial1.print("Altitude Value\n");
    Serial1.printf("Current Height: %d\n",sonar_alt);
    Serial1.printf("Desired Height: %d\n",altitudeDesired);
    Serial1.printf("PID Correction: %d\n",correctionAltitude);
    Serial1.print("------------------\n");
    Serial1.print("Velocity Value\n");
    Serial1.printf("Desired velocity: x:%f y:%f\n",desiredVelocityX,desiredVelocityY);
    Serial1.printf("Velocity Error: x:%f y:%f\n",newVelocityErrorX,newVelocityErrorY);
    Serial1.printf("PID Correction: x:%d y:%d\n",correctionX,correctionY);
    Serial1.print("------------------\n");
    Serial1.printf("STATE: %d\n",STATE);
    Serial1.print("------------------\n");
    break;
    
  case 'r':
    motors.armed(true);
    if(motors.armed())
    {
      Serial1.printf("Motors Armed! \n");
    }
    else
    {
      Serial1.printf("Motors Not Armed \n");
    }
    break;
    
  case 't':
    motors.armed(false);
    if(motors.armed())
    {
      Serial1.printf("Motors Armed! \n");
    }
    else
    {
      Serial1.printf("Motors Not Armed \n");
    }
    break;
    
  case 'i':
    Serial1.printf("Auto takeoff \n");
    throttleValue = 1460;
    g.rc_3.set_pwm(throttleValue);
    rollValue = g.rc_1.radio_in;
    pitchValue = g.rc_2.radio_in;   

    //gaan naar state autotakeoff.
    STATE = AUTOTAKEOFF;  
    break;
    
  case 'u':
    Serial1.printf("Auto land \n");
    throttleValue = g.rc_3.radio_in;
    STATE = AUTOLAND;
    break;
    
  case 'y':
    if(opticalFlow){      
      Serial1.printf("Desactivate optical flow\n");
      opticalFlow = false;
    } else {
      Serial1.printf("Activate optical flow\n");
      //initialise PI and position
      rollValue = g.rc_1.radio_in;
      pitchValue = g.rc_2.radio_in;
      
      integralVelocityErrorX = 0.0;
      integralVelocityErrorY = 0.0;
      correctionX = 0;
      correctionY = 0;     
      positionX = 0.0;
      positionY = 0.0;
      desiredVelocityX = 0.0;
      desiredVelocityY = 0.0;
      
      opticalFlow = true;
    }    
    break;
    
  case 'm':
    // Here we read out the x and y errors
    //Serial1.println(readString);
    
    //get optical displacement X
    index = 1;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(0,index-1).toCharArray(charBuf, 7);
    //Serial1.println(charBuf);
    opticalDisplacementX = atof(charBuf);
    
    //get optical displacement y
    tempIndex = index;
    index++;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(tempIndex,index-1).toCharArray(charBuf, 7);
    //Serial1.println(charBuf);
    opticalDisplacementY = atof(charBuf);
 
    //get timediff
    tempIndex = index;
    index++;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(tempIndex,index-1).toCharArray(charBuf, 7);
    //Serial1.println(charBuf);
    timeDiff = atoi(charBuf);
    
        
    //opticalDisplacement moet gecompenseerd worden met roll en pitch
    newPitch = (long)ahrs.pitch_sensor;
    
    //let op wanneer 360° <-> 0° overslaat
    //newPitch is overgeslaan van 0° -> 360°
    if(newPitch > 26000L && oldPitch < 10000L){
      pitchDiff = ((float)(newPitch - oldPitch - 36000L))/62.0*320.0/100.0;
    }
    //new Pitch is overgeslaan van 360° -> 0°
    else if(newPitch < 10000L && oldPitch > 26000L){
      pitchDiff = ((float) (newPitch - oldPitch + 36000L))/62.0*320.0/100.0;
    } else {
      pitchDiff = ((float) (newPitch - oldPitch))/62.0*320.0/100.0;  
    }
    
    pitchDiffT4 = pitchDiffT3;
    pitchDiffT3 = pitchDiffT2;
    pitchDiffT2 = pitchDiffT1;
    pitchDiffT1 = pitchDiffT0;
    pitchDiffT0 = pitchDiff;
    
    oldPitch = newPitch;
    
    newRoll = (long)ahrs.roll_sensor;
    
    //let op wanneer 360° <-> 0° overslaat
    //newRoll is overgeslaan van 0° -> 360°
    if(newRoll > 26000L && oldRoll < 10000L){
      rollDiff = ((float)(newRoll - oldRoll - 36000L))/54.0*240.0/100.0;
    }
    //new Roll is overgeslaan van 360° -> 0°
    else if(newRoll < 10000L && oldRoll > 26000L){
      rollDiff = ((float) (newRoll - oldRoll + 36000L))/54.0*240.0/100.0;
    } else {
      rollDiff = ((float) (newRoll - oldRoll))/54.0*240.0/100.0;  
    }
    
    rollDiffT4 = rollDiffT3;
    rollDiffT3 = rollDiffT2;
    rollDiffT2 = rollDiffT1;
    rollDiffT1 = rollDiffT0;
    rollDiffT0 = rollDiff;
    
    oldRoll = newRoll;
      
    //Omzetting van pixels naar meter (320pixels = 1,2 meter(op hoogte 1m) | 320 pixels = 0,6meter (op hoogte 0,5m)
    //We werken met centimeter voor de hoogte, dus ook voor de afstand.
    displacementX = (opticalDisplacementX-pitchDiffT2)*1.2/360.0*((float)sonar_alt);
    displacementY = (opticalDisplacementY+rollDiffT2)*1.02/240.0*((float)sonar_alt);
    
    //Bereken nieuwe positie   
    positionX = positionX + displacementX; 
    positionY = positionY + displacementY;
    
    //timediff is in miliseconden, versplaatsing in cm --> snelheid in meter per seconde: doe maal 10
    velocityX = 10.0*displacementX/((float)timeDiff);
    velocityY = 10.0*displacementY/((float)timeDiff);

    //als opticalFlow = true
    if(STATE == ALTHOLD && opticalFlow)
    { 
      //only perform PIVelocity when new opticalFlow data is available!      
      PIVelocity();
      
      //de radio signalen aanpassen, blijkt niet te werken, waarschijnlijk omdat ze verkeerd worden geïnitialiseerd door afwezigheid van een radioontvangen met bijhorende afstandsbediening
      
      //pitch correction
      g.rc_2.radio_in = (signed int) pitchValue + (signed int) correctionX;
      g.rc_2.set_pwm(g.rc_2.radio_in);
      
      //roll correction
      g.rc_1.radio_in = (signed int) rollValue - (signed int) correctionY;
      g.rc_1.set_pwm(g.rc_1.radio_in);     
    
      
      //here a function generator for position will be necessary?
      
      // TODO ///
    }
    
    //dump data to RPi
    //but first get the yaw
    if(newYaw == 0 && oldYaw == 0){
      newYaw = (long)ahrs.yaw_sensor;
      oldYaw = newYaw;
    } else {
      newYaw = (long)ahrs.yaw_sensor;
    }
    
    //let op wanneer 360° -> 0° overslaat
    //newYaw is overgeslaan van 0° -> 360°
    if(newYaw > 26000L && oldYaw < 10000L){
      yawDiff = (int)(newYaw - oldYaw - 36000L);
    }
    //new Yaw is overgeslaan van 360° -> 0°
    else if(newYaw < 10000L && oldYaw > 26000L){
      yawDiff = (int) (newYaw - oldYaw + 36000L);
    } else {
      yawDiff = (int) (newYaw - oldYaw);    
    }
    
    yawDiffT4 = yawDiffT3;
    yawDiffT3 = yawDiffT2;
    yawDiffT2 = yawDiffT1;
    yawDiffT1 = yawDiffT0;
    yawDiffT0 = yawDiff;
    
    oldYaw = newYaw;
    
    Serial1.printf("SLAM|%d|%d|%d|%d|\n", (int)displacementX, (int)displacementY, timeDiff, yawDiffT3);

    break;
  
  case 'o':
    // Set Sonar PID
    Serial.println(readString);
    
    index = 1;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(0,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    pOptical = atof(charBuf);
    tempIndex = index;
    index++;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(tempIndex,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    dOptical = atof(charBuf);
    tempIndex = index;
    index++;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(tempIndex,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    iOptical = atof(charBuf);
    
    Serial1.printf("got p: %f, d: %f, i: %f\n",pOptical,dOptical,iOptical);
  break;
    
  case 'k':
    // Set Sonar PID
    Serial.println(readString);
    
    index = 1;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(0,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    pAltitude = atof(charBuf);
    tempIndex = index;
    index++;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(tempIndex,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    dAltitude = atof(charBuf);
    tempIndex = index;
    index++;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(tempIndex,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    iAltitude = atof(charBuf);
    
    Serial1.printf("got p: %f, d: %f, i: %f\n",pAltitude,dAltitude,iAltitude);
  break;
  
  
  case 'l':
    // Set liftoffThreshold
    Serial.println(readString);
    
    index = 1;
    while(readString.substring(index-1,index) != "|")
    {
      index++;
    }
    readString.substring(0,index-1).toCharArray(charBuf, 10);
    //Serial.println(charBuf);
    liftoffThreshold = atof(charBuf);
        
    Serial1.printf("got liftoffThreshold: %f\n", liftoffThreshold);
  break;

  default: 
    // Say what you got:
    Serial1.print("I received: ");
    Serial1.print(readString);
    Serial.print("I received: ");
    Serial.print(readString);
    //Serial.print("No action taken\n");
    //Serial1.print("No action taken\n");
    break;
  }
  readString = 0;
}
