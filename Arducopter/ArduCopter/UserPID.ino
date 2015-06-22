void PIDAltitude()
{
  // PID control for Altitiude Hold
  //------------------------------------------------------------------------------------
  //newAltError = next_WP.alt - sonar_alt;
  
  newAltitudeError = ((int)altitudeWayPoint) - sonar_alt;
  
  derivativeAltitudeError = (newAltitudeError - oldAltitudeError)*20.0;
 
  integralAltitudeError = integralAltitudeError + newAltitudeError*0.05;
  
  correctionAltitude = (signed int)(newAltitudeError*pAltitude + derivativeAltitudeError*dAltitude + integralAltitudeError*iAltitude);
  
  //Update Old Altitude Error
  oldAltitudeError = newAltitudeError;
}

void PIVelocity()
{  
  // snelheid moet geregeld worden
  newVelocityErrorX = desiredVelocityX - velocityX;
  newVelocityErrorY = desiredVelocityY - velocityY;
  
  integralVelocityErrorX = integralVelocityErrorX + 0.001*newVelocityErrorX*((float)timeDiff);
  integralVelocityErrorY = integralVelocityErrorY + 0.001*newVelocityErrorY*((float)timeDiff);
  
  correctionX = (signed int)(newVelocityErrorX*pOptical);
  correctionY = (signed int)(newVelocityErrorY*pOptical);
  
  // de correctie moet binnen de perken blijven, anders wordt het gedrag van de quadcopter te extreem
  //correctionX = constrain(correctionX, -500, 500);
  //correctionY = constrain(correctionY, -500, 500);
}
