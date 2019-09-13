/******************************************************************************
   Debug
******************************************************************************/

void writeDebugOutputDateTime(DateTime time) {
  Serial.print(time.year(), DEC);
  Serial.print('/');
  Serial.print(time.month(), DEC);
  Serial.print('/');
  Serial.print(time.day(), DEC);
  Serial.print(' ');
  writeDebugOutputTime(time);
}

void writeDebugOutputTime(DateTime time) {
  Serial.print(time.hour(), DEC);
  Serial.print(':');
  Serial.print(time.minute(), DEC);
  Serial.print(':');
  Serial.print(time.second(), DEC);
}

/******************************************************************************
    Output to serial monitor
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
    Precondition:
      global variables in current states
    Postcondition:
      values of various variables written to serial monitor, optionally
      fitting format for Arduino serial plotter
*/

void writeDebugOutput() {
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print("X,");
#endif
#ifdef SERIAL_DEBUG_RAW_SENSOR
  Serial.print(sensorXRawRef);
  Serial.print(",");
  Serial.print(sensorX1RawAbs);
  Serial.print(",");
  Serial.print(sensorX2RawAbs);
  Serial.print(",");
#endif
  Serial.print(sensorX1Level);
  Serial.print(",");
  Serial.print(sensorX2Level);
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Diff,");
#endif
  Serial.print(sensorXDiff);
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(",");
  Serial.print(" Ok,");
  Serial.print(sensorLevelsOkX);
#endif

#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" | Y,");
#endif
#ifdef SERIAL_DEBUG_RAW_SENSOR
  Serial.print(sensorYRawRef);
  Serial.print(",");
  Serial.print(sensorY1RawAbs);
  Serial.print(",");
  Serial.print(sensorY2RawAbs);
  Serial.print(",");
#endif
  Serial.print(sensorY1Level);
  Serial.print(",");
  Serial.print(sensorY2Level);
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Diff,");
#endif
  Serial.print(sensorYDiff);
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(",");
  Serial.print(" Ok,");
  Serial.print(sensorLevelsOkY);
#endif

  Serial.print(",| trackON,");
  Serial.print(trackingActive);

  /* RTC */
  Serial.print(", ");
  writeDebugOutputTime(actualTime);

  /* Sun Position */
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" target X,");
#endif
  Serial.print(targetPositionX);
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Y,");
#endif
  Serial.print(targetPositionY);

  // Position estimate
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Pos X,");
#endif
  Serial.print(actualPositionX);
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Y,");
#endif
  Serial.print(actualPositionY);

  /* Position error
    Serial.print(",");
    #ifndef SERIAL_PLOTTER_FORMAT
    Serial.print(" Err X,");
    #endif
    Serial.print(errorPositionX);
    Serial.print(",");
    #ifndef SERIAL_PLOTTER_FORMAT
    Serial.print(" Y,");
    #endif
    Serial.print(errorPositionY);
  */

  /* Motor command */
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print("| MotorCmd X,");
#endif
  Serial.print(motorSettingX);
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Y,");
#endif
  Serial.print(motorSettingY);

  /* Motor currents */
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print("| MotorI X,");
#endif
  Serial.print(motorCurrentX);
  Serial.print(",");
#ifndef SERIAL_PLOTTER_FORMAT
  Serial.print(" Y,");
#endif
  Serial.print(motorCurrentY);

  /* Travel limits
    Serial.print(",");
    #ifndef SERIAL_PLOTTER_FORMAT
    Serial.print("| Lim XMax,");
    #endif
    Serial.print(atLimitXMax);
    Serial.print(",");
    #ifndef SERIAL_PLOTTER_FORMAT
    Serial.print(" XMin,");
    #endif
    Serial.print(atLimitXMin);
    Serial.print(",");
    #ifndef SERIAL_PLOTTER_FORMAT
    Serial.print(" YMax,");
    #endif
    Serial.print(atLimitYMax);
    Serial.print(",");
    #ifndef SERIAL_PLOTTER_FORMAT
    Serial.print(" YMin,");
    #endif
    Serial.print(atLimitYMin);
  */

#ifndef SERIAL_PLOTTER_FORMAT
  
  /* User inputs
    Serial.print(",| safe,");
    Serial.print(safePositionCmd);
    Serial.print(", trackingRequested, ");
    Serial.print(trackingRequested);
    Serial.print(", manual X+,");
    Serial.print(manualMoveXPlus);
    Serial.print(", X-,");
    Serial.print(manualMoveXMinus);
    Serial.print(", Y+,");
    Serial.print(manualMoveYPlus);
    Serial.print(", Y-,");
    Serial.print(manualMoveYMinus);
  */

  Serial.print(",|  CLActive X,");
  Serial.print(closedLoopActiveX);
  Serial.print(", Y,");
  Serial.print(closedLoopActiveY);

  Serial.print(";  Faults: gen,");
  Serial.print(generalFault);
  Serial.print(", CLX,");
  Serial.print(closedLoopFaultX);
  Serial.print(", CLY,");
  Serial.print(closedLoopFaultY);
#endif
  Serial.println("");   // end debug line
}
