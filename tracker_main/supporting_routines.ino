/******************************************************************************
   supporting routines for application control logic
******************************************************************************/

/******************************************************************************
    Control Motors based on user input
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
    Precondition:
      manualMove* global variables updated
    Postcondition:
      motors enabled or disabled appropriately
*/
inline void manualMotorControl() {
  // Check manual commands
  userInputFault = false;
  if (manualMoveXPlus) {
    if (!atLimitXMax) {
      driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_FAST);
    } else {
      userInputFault = true;
    }
  } else if (manualMoveXMinus) {
    if (!atLimitXMin) {
      driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_FAST);
    } else {
      userInputFault = true;
    }
  } else {
    driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  }
  if (manualMoveYPlus) {
    if (!atLimitYMax) {
      driveY(MOTOR_DIR_PLUS, MOTOR_SPEED_FAST);
    } else {
      userInputFault = true;
    }
  } else if (manualMoveYMinus) {
    if (!atLimitYMin) {
      driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_FAST);
    } else {
      userInputFault = true;
    }
  } else {
    driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  }
  if (userInputFault) {
    driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
    driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
  }
}

/******************************************************************************
    Control Motors based on sensor feedback
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
    Precondition:
    TODO
    Postcondition:
      motors enabled or disabled appropriately
*/
inline void closedLoopMotorControl() {
  // X tracking --------------
  // TODO: maximum temperature control: if at limit, do not enter closed loop?
  if (sensorLevelsOkX && !safePositionCmd && !closedLoopFaultX && sensorInitDone) {
    closedLoopActiveX = true;
    if (generalFault) {
      closedLoopActiveX = false;
      driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_OFF);
    } else if (trackingActive) {
      errorClosedLoopX = sensorXDiff;
      trackingLockedX = false;
      if (SENSOR_POLARITY_FLIP_X) {
        errorClosedLoopX *= -1;
      }
      if ((errorClosedLoopX > CLOSED_LOOP_FAST_MOVE_DEADBAND_X) && !atLimitXMin) {
        driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_FAST);
        trackingStableCounterX = 0;       // reset counter if not locked
      } else if ((errorClosedLoopX > CLOSED_LOOP_CONTROL_DEADBAND_X) && !atLimitXMin) {
        driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_SLOW);
        trackingStableCounterX = 0;       // reset counter if not locked
      } else if ((errorClosedLoopX < (-1 * CLOSED_LOOP_FAST_MOVE_DEADBAND_X)) && !atLimitXMax) {
        driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_FAST);
        trackingStableCounterX = 0;       // reset counter if not locked
      } else if ((errorClosedLoopX < (-1 * CLOSED_LOOP_CONTROL_DEADBAND_X)) && !atLimitXMax) {
        driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_SLOW);
        trackingStableCounterX = 0;       // reset counter if not locked
      } else {
        driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_OFF);
        if (atLimitXMin || atLimitXMax) {
          //TODO handle error
        } else if (++trackingStableCounterX >= TRACKING_STABLE_COUNT) {
          trackingLockedX = true;
          trackingStableCounterX--;       // to maintain stable count despite incrementing above
        }
      }
      // sanity check closed loop position against open loop estimate
#if CLOSED_LOOP_POSITIONING_SANITY_CHECK
      if (abs(errorPositionX) > CLOSED_LOOP_ABORT_LIMIT_X) {
        closedLoopActiveX = false;
        closedLoopFaultX = true;
      }
#endif
    }
  } else {          // not closedLoopActiveX
    //weak sensor levels or safe mode command , not tracking, Use open loop.
    closedLoopActiveX = false;
    // keep command for safe position and tracking mode
    if (!safePositionCmd && !trackingActive) {
      driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
    }
  }

  // Y tracking --------------
  if (sensorLevelsOkY && !safePositionCmd && !closedLoopFaultY && Y_AXIS_ACTIVE && sensorInitDone) {
    closedLoopActiveY = true;
    if (generalFault) {
      closedLoopActiveY = false;
      driveY(MOTOR_DIR_PLUS, MOTOR_SPEED_OFF);
    } else if (trackingActive) {
      errorClosedLoopY = sensorYDiff;
      if (SENSOR_POLARITY_FLIP_Y) {
        errorClosedLoopY *= -1;
      }
      trackingLockedY = false;
      if ((errorClosedLoopY > CLOSED_LOOP_FAST_MOVE_DEADBAND_Y) && !atLimitYMin) {
        driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_FAST);
        trackingStableCounterY = 0;       // reset counter if not locked
      } else if (errorClosedLoopY > CLOSED_LOOP_CONTROL_DEADBAND_Y && !atLimitYMin) {
        driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_SLOW);
        trackingStableCounterY = 0;       // reset counter
      } else if ((errorClosedLoopY < (-1 * CLOSED_LOOP_FAST_MOVE_DEADBAND_Y)) && !atLimitYMax) {
        driveY(MOTOR_DIR_PLUS, MOTOR_SPEED_FAST);
        trackingStableCounterY = 0;       // reset counter if not locked
      } else if ((errorClosedLoopY < (-1 * CLOSED_LOOP_CONTROL_DEADBAND_Y)) && !atLimitYMax) {
        driveY(MOTOR_DIR_PLUS, MOTOR_SPEED_SLOW);
        trackingStableCounterY = 0;       // reset counter
      } else {
        driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
        if (atLimitYMin || atLimitYMax) {
          //TODO handle error
        } else if (++trackingStableCounterY >= TRACKING_STABLE_COUNT) {
          trackingLockedY = true;
          trackingStableCounterY--; // to maintain stable count
        }
      }
      // sanity check closed loop position against open loop estimate
#if CLOSED_LOOP_POSITIONING_SANITY_CHECK
      if (abs(errorPositionY) > CLOSED_LOOP_ABORT_LIMIT_Y) {
        closedLoopActiveY = false;
        closedLoopFaultY = true;
      }
#endif
    }
  } else {          // not closedLoopActiveY
    //weak sensor levels or safe mode command, not tracking, Use open loop.
    closedLoopActiveY = false;
    // keep command for safe position and tracking mode
    if (!safePositionCmd && !trackingActive) {
      driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
    }
  }
}

inline void openLoopMotorControl() {
  if (!closedLoopActiveX && rtcOK && OPEN_LOOP_POSITIONING_ENABLE) {
    // aim roughly towards the sun using open loop position estimate
    if (sensorInitDone && (trackingActive || safePositionCmd)) {
      if (!positionInitialized) {
        initializePositionAtLimits();
      }
      if (errorPositionX > OPEN_LOOP_FAST_MOVE_DEADBAND_X && !atLimitXMin) {
        driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_FAST);
      } else if (errorPositionX > OPEN_LOOP_CONTROL_DEADBAND_X && !atLimitXMin) {
        driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_SLOW);
      } else if (errorPositionX < -OPEN_LOOP_FAST_MOVE_DEADBAND_X && !atLimitXMax) {
        driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_FAST);
      } else if (errorPositionX < -OPEN_LOOP_CONTROL_DEADBAND_X && !atLimitXMax) {
        driveX(MOTOR_DIR_PLUS, MOTOR_SPEED_SLOW);
      } else {
        driveX(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
      }
    } else {
      // leave motor command alone , do not override manual or closed loop command
    }   //endif control wanted
  }   //endif X open loop capable

  if (!closedLoopActiveY && rtcOK && Y_AXIS_ACTIVE && OPEN_LOOP_POSITIONING_ENABLE) {
    if (sensorInitDone && (trackingActive || safePositionCmd)) {
      if (!positionInitialized) {
        initializePositionAtLimits();
      }
      if (errorPositionY > OPEN_LOOP_FAST_MOVE_DEADBAND_Y && !atLimitYMin) {
        driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_FAST);
      } else if (errorPositionY > OPEN_LOOP_CONTROL_DEADBAND_Y && !atLimitYMin) {
        driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_SLOW);
      } else if (errorPositionY < -OPEN_LOOP_FAST_MOVE_DEADBAND_Y && !atLimitYMax) {
        driveY(MOTOR_DIR_PLUS, MOTOR_SPEED_FAST);
      } else if (errorPositionY < -OPEN_LOOP_CONTROL_DEADBAND_Y && !atLimitYMax) {
        driveY(MOTOR_DIR_PLUS, MOTOR_SPEED_SLOW);
      } else {
        driveY(MOTOR_DIR_MINUS, MOTOR_SPEED_OFF);
      }
    } else {
      // leave motor command alone , do not override manual or closed loop command
    }   //endif control wanted
  }   //endif Y open loop capable
}
