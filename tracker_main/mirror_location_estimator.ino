/******************************************************************************
   Mirror location estimator
******************************************************************************/

/******************************************************************************
    Updates position estimates, assuming that the actual output states
    have run the actuator for the specified period.

    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      elapsedTime in ms
    Precondition:
    Postcondition:
      actualPositionX: in degrees [0..360) updated
      actualPositionY: in degrees [-90..90) updated
*/
void updatePosition(int elapsedTime) {
  float delta;
  switch (motorSettingX) {
    case 2:
    case -2:
      delta = MOTOR_SPEED_FAST_PER_SEC_X;
      break;
    case 1:
    case -1:
      delta = MOTOR_SPEED_SLOW_PER_SEC_X;
      break;
    default:
    case 0:
      delta = 0;
      break;
  }
  delta = (delta * elapsedTime) / 1000;   // distance = speed * time

  if (motorSettingX > 0) {
    actualPositionX += delta;
  } else {
    actualPositionX -= delta;
  }

  // Range wraparound
  if (actualPositionX >= 180) {
    actualPositionX -= 360;
  } else if (actualPositionX < -180) {
    actualPositionX += 360;
  }

  if (Y_AXIS_ACTIVE) {
    switch (motorSettingY) {
      case 2:
      case -2:
        delta = MOTOR_SPEED_FAST_PER_SEC_Y;
        break;
      case 1:
      case -1:
        delta = MOTOR_SPEED_SLOW_PER_SEC_Y;
        break;
      default:
      case 0:
        delta = 0;
        break;
    }
    delta = (delta * elapsedTime) / 1000;   // distance = speed * time
    if (motorSettingY > 0) {
      actualPositionY += delta;
    } else {
      actualPositionY -= delta;
    }

    // Range check
    if (actualPositionY > 90) {
      // TODO
    } else if (actualPositionY < -90) {

    }
  }
}

/******************************************************************************
    initialize position by travelling to home position, based on feedback
    from the limit switches.

    This function blocks execution until it is complete.

    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      Y_AXIS_ACTIVE
    Precondition:
    Postcondition:
      array moved to configured home position.
      actualPositionX: in degrees [0..360) updated
      actualPositionY: in degrees [-90..90) updated
      positionInitialized true when successful.
*/
void initializePositionAtLimits() {
  bool* p_limitX;     // points to limit boolean we will be using for X
  bool* p_limitY;     // points to limit boolean we will be using for Y
  bool dirTowardsLimitX;  // direction to drive toward the limit for X
  bool dirTowardsLimitY;  // direction to drive toward the limit for Y

  if (HOME_AT_MAX_X) {
    p_limitX = &atLimitXMax;
    dirTowardsLimitX = MOTOR_DIR_PLUS;
    Serial.print(" Calibrating X at max");
  } else {
    p_limitX = &atLimitXMin;
    dirTowardsLimitX = MOTOR_DIR_MINUS;
    Serial.print(" Calibrating X at min");
  }
  if (HOME_AT_MAX_Y) {
    p_limitY = &atLimitYMax;
    dirTowardsLimitY = MOTOR_DIR_PLUS;
    Serial.print(", Y at max");
  } else {
    p_limitY = &atLimitYMin;
    dirTowardsLimitY = MOTOR_DIR_MINUS;
    Serial.print(", Y at min");
  }
  Serial.println("");

  // 0. if limit switch active, move away from the switch
  Serial.println("initializePosition 0: fast away from limit");
  moveAwayFromLimits(p_limitX, p_limitY, dirTowardsLimitX, dirTowardsLimitY, MOTOR_SPEED_FAST);
  delay(MOVE_STOP_FROM_FAST_DELAY);

  // 1. do fast move to quickly find limit switch
  Serial.println("initializePosition 1: fast towards limit");
  readTravelLimits();
  readUserInputs();
  while ((!*p_limitX || (!*p_limitY && Y_AXIS_ACTIVE)) && (trackingRequested || safePositionCmd)) {
    readTravelLimits();
    Serial.print(", Lim X ");
    Serial.print(*p_limitX);
    if (!*p_limitX) {
      driveX(dirTowardsLimitX, MOTOR_SPEED_FAST);
    } else {
      Serial.print(" Xlimit met");
      driveX(dirTowardsLimitX, MOTOR_SPEED_OFF);
    }
    if (Y_AXIS_ACTIVE) {
      Serial.print(", Y ");
      Serial.print(*p_limitY);
      if (!*p_limitY) {
        driveY(dirTowardsLimitY, MOTOR_SPEED_FAST);
      } else {
        Serial.print(" Ylimit met");
        driveY(dirTowardsLimitY, MOTOR_SPEED_OFF);
      }
    }
    Serial.println("");
    delay(100);
    readUserInputs();
  }
  driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  delay(MOVE_STOP_FROM_FAST_DELAY);

  //debug: see if we overran the limit switches
  readTravelLimits();
  if ((!*p_limitX || (!*p_limitY && Y_AXIS_ACTIVE)) && (trackingRequested || safePositionCmd)) {
    Serial.println("min limits not set after homing");
    generalFault = true;
    // TODO add code to put us back on the limits?
  }

  // 2. do slow move to move away from limit switch
  Serial.println("initializePosition 2: slow away from limit");
  moveAwayFromLimits(p_limitX, p_limitY, dirTowardsLimitX, dirTowardsLimitY, MOTOR_SPEED_SLOW);
  delay(MOVE_STOP_FROM_SLOW_DELAY);

  if (trackingRequested || safePositionCmd) {     // we did not abort due to user turning off
    // set current position estimate to known home position values
    actualPositionX = HOME_POSITION_X;
    actualPositionY = HOME_POSITION_Y;
    positionInitialized = true;
    Serial.println("position initialized at home");
  }
}

/******************************************************************************
    Perform move away from the limit switches at the given motor speed

    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      p_limitX: pointer to bool: limit switch of interest for X
      p_limitY: pointer to bool: limit switch of interest for Y
      dirTowardsLimitX: motor direction to move towards limit of interest in X
      dirTowardsLimitY: motor direction to move towards limit of interest in Y
      motorSpeed: one of MOTOR_SPEED_XXX
    Precondition:
      trackingRequested and safePositionCmd represents actual user input
    Postcondition:
      array moved away from limit switches
*/
void moveAwayFromLimits(bool* p_limitX, bool* p_limitY,
                        bool dirTowardsLimitX, bool dirTowardsLimitY, int motorSpeed) {
  readTravelLimits();
  readUserInputs();
  // TODO limit infinite loop
  while ((*p_limitX || (*p_limitY && Y_AXIS_ACTIVE)) && (trackingRequested || safePositionCmd))
  {
    readTravelLimits();
    Serial.print(", Lim X ");
    Serial.print(*p_limitX);
    if (*p_limitX) {
      driveX(!dirTowardsLimitX, motorSpeed);
    } else {
      Serial.print(" Xlimit off");
      driveX(!dirTowardsLimitX, MOTOR_SPEED_OFF);
    }
    if (Y_AXIS_ACTIVE) {
      Serial.print(", Y ");
      Serial.print(*p_limitY);
      if (*p_limitY) {
        driveY(!dirTowardsLimitY, motorSpeed);
      } else {
        Serial.print(" Ylimit off");
        driveY(!dirTowardsLimitY, MOTOR_SPEED_OFF);
      }
    }
    Serial.println("");
    delay(100);
    readUserInputs();
  }
  driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
}
