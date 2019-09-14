/******************************************************************************
   UI
******************************************************************************/

/**
    Updates the status LED
    by:     Gregory Fung <gwfung@gmail.com>

    Precondition:
        sensorLevelsOkX
        trackingActive
        closedLoopActiveX
        closedLoopFaultX,
        sensorLevelsOkY
        closedLoopActiveY
        closedLoopFaultY,
        safePositionCmd
        rtcOK
        userInputFault
        generalFault
    Postcondition:
      LED driver set to give the required status code
        tracking: steady Green
        open loop tracking: 0.5Hz blinking Green
        safePositionCmd active: yellow
        any fault: red
*/
void setStatusLed() {
  ledBlink = LED_BLINK_STEADY;
  if ((safePositionCmd && !OPEN_LOOP_POSITIONING_ENABLE) || userInputFault || !rtcOK) {
    // safe position requires open loop positioing capability
    // User input error
    ledColour = LED_COLOUR_RED;
  } else if (generalFault || closedLoopFaultX || (Y_AXIS_ACTIVE && closedLoopFaultY)) {
    ledColour = LED_COLOUR_RED;
    ledBlink = LED_BLINK_FAST;
  } else if (sensorInitDone && sensorLevelsOkX && (!Y_AXIS_ACTIVE || sensorLevelsOkY) &&
             !safePositionCmd) {
    ledColour = LED_COLOUR_GREEN;
    if (!trackingActive) {
      ledBlink = LED_BLINK_SLOW;
    }
  } else {
    ledColour = LED_COLOUR_YELLOW;
    if (!trackingActive || safePositionCmd) {
      ledBlink = LED_BLINK_SLOW;
    }
  }
}

/******************************************************************************
    Updates the LED state based on desired status. controls blinking.
    based on https://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
    Precondition:
      ledBlinkInterval:
      ledColour: one of the LED_COLOUR_XXX values
      ledBlink: 0: steady, 1: 0.5Hz, 2: 2Hz
      ledActualState: true: LED currently on
    Postcondition:
      LED state reflects status variables
      ledBlink updated
      ledActualColour reflects LED
*/
void ledStateUpdate() {
  // TODO: support 2Hz blinking

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();
  bool blinkingDue = (currentMillis - previousLedMillis) >= ledBlinkInterval;

  // react immediately if: 1. coming out of blinkmode, led is low or 2. colour change
  if ((!ledBlink && ledActualState == LOW) || (ledActualColour != ledColour) || blinkingDue) {
    // save the last time you updated the LED
    previousLedMillis = currentMillis;
    if (blinkingDue) {
      // if the LED is off or steady mode, turn it on
      if (ledActualState == LOW || !ledBlink) {
        ledActualState = HIGH;
      } else {
        ledActualState = LOW;
      }
    } else {
      ledActualState = HIGH;
    }
    ledActualColour = ledColour;
    writeBiColourLedPins(DO_STATUS_LED_RED, DO_STATUS_LED_GREEN, ledColour, ledActualState);
  }
}
