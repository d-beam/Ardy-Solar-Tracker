/******************************************************************************
   Main scan loop for Solar Tracker
   by:      Gregory Fung <gwfung@gmail.com>
*/

void loop() {
  int testCount = 0;
  static bool lastSafePositionCmd = false;

  lastSafePositionCmd = safePositionCmd;
  readUserInputs();

  // calculations that are only needed infrequently
  if (--slowUpdateCountdown <= 0 || (lastSafePositionCmd != safePositionCmd)) {
    slowUpdateCountdown = EXTENDED_COMPUTATION_COUNT;
    actualTime = timeNow();
    if (safePositionCmd) {
      // TODO time dependent safe position?
      targetPositionX = SAFE_POSITION_X;
      targetPositionY = SAFE_POSITION_Y;
    } else {
      calculateSunLocation(actualTime, sunPosition, LONGITUDE, LATITUDE,
                           &targetPositionX, &targetPositionY);

      // save possibly intermediate results
      targetAzimuth = targetPositionX;
      targetElevation = targetPositionY;

      /* RTC */
      writeDebugOutputTime(actualTime);

      /* Sun Position */
#ifdef SERIAL_DEBUG_SUN_CALC
      Serial.print(", Sun at Azi ");
      Serial.print(targetAzimuth);
      Serial.print(", Elev ");
      Serial.print(targetElevation);
#endif
      if (EQUATORIAL_AXES_MODE) {
        convertToStableEquatorialAxes(LATITUDE, targetAzimuth, targetElevation, &targetPositionX, &targetPositionY);
      }
#ifdef SERIAL_DEBUG_SUN_CALC
      Serial.println("");
#endif
    }
    updatePosition(EXTENDED_COMPUTATION_MS);
  }
  errorPositionX = actualPositionX - targetPositionX;
  errorPositionY = actualPositionY - targetPositionY;

  // Can we track right now?
  trackingActive  = trackingRequested;
  if (targetElevation < SUN_ELEVATION_MIN_FOR_TRACKING) {
    trackingActive = false;
  }
  if (EQUATORIAL_AXES_MODE) {
    if ((targetPositionY < SUN_HOUR_ANGLE_MIN_FOR_TRACKING) ||
        (targetPositionY > SUN_HOUR_ANGLE_MAX_FOR_TRACKING)) {
      trackingActive = false;
    }
  }

  // TODO: maximum temperature control: if at limit, aim for non-optimal position by
  // adding offset to targetPositionX?

  // reading Sensors -------------------------------------------------------
  readTrackingSensorsX();
  sensorLevelsOkX = CheckSensorLevels(&sensorXRawRef, SENSOR_REF_DEFAULT,
                                      &sensorX1RawAbs, &sensorX2RawAbs, &sensorXRawDiff,
                                      &sensorX1Level, &sensorX2Level, &sensorXDiff,
                                      SENSOR_ABS_MIN, SENSOR_ABS_MAX);
  readTrackingSensorsY();
  sensorLevelsOkY = CheckSensorLevels(&sensorYRawRef, SENSOR_REF_DEFAULT,
                                      &sensorY1RawAbs, &sensorY2RawAbs, &sensorYRawDiff,
                                      &sensorY1Level, &sensorY2Level, &sensorYDiff,
                                      SENSOR_ABS_MIN, SENSOR_ABS_MAX);

  // Closed loop and manual control-------------------------------------------------------
  readTravelLimits();
  if (!trackingRequested && !safePositionCmd) {
    manualMotorControl();
  } else {
    closedLoopMotorControl();
  } //endif manual or closed loop automatic

  // open loop position estimate calibration:
  // if trackingLocked, we could update our position estimate to the actual expected position of the sun?
  // this is OK, possibly except:
  // 1. where there is a fault of a "false sun" that we have tracked (ie reflection?)
  if (trackingLockedX && (trackingLockedY || !Y_AXIS_ACTIVE)) {
    // update estimated actual position with expected position
    actualPositionX = targetPositionX;
    actualPositionY = targetPositionY;
    positionInitialized = true;
    //Serial.println("position initialized using tracking lock and expected sun position");
  }

  //Open loop positioning ---------------------------------------------------
  readTravelLimits();
  if (generalFault) {
    // unexpected?  TODO improve handling
    driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  } else {
    openLoopMotorControl();
  }   //endif generalFault

  // Fault handling ---------------------------------------------------
  if (!areMotorsOK()) {
    // TODO better handling
    generalFault = true;
  }

  if (closedLoopFaultX) {
    //Reinitialize? up to 3 times?
  }
  if (closedLoopFaultY) {
    //Reinitialize? up to 3 times?
  }

  // System maintanence ---------------------------------------------------
  setStatusLed();
  ledStateUpdate();
  updatePosition(UPDATE_DELAY_MS + NORMAL_COMPUTATION_MS);
  writeDebugOutput();
  delay(UPDATE_DELAY_MS);
}
