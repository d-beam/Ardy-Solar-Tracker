/******************************************************************************
   I/Os
******************************************************************************/

/******************************************************************************
    Reads user inputs and updates global variables.
    by:     Greg

    Precondition:
    Postcondition:
      trackingRequested: true = yes
      safePositionCmd: true = active
      manualMoveXPlus: true = active
      manualMoveXMinus: true = active
      manualMoveYPlus: true = active
      manualMoveYMinus: true = active
        After each input, short dead time before the mode can be changed.

*/
void readUserInputs() {
  // TODO add debouncing
  trackingRequested  = !digitalRead(DI_MODE_TRACK);  //pull-up resistor, switch pulls voltage down at limit
  safePositionCmd  = !digitalRead(DI_MODE_SAFE);

  if (MANUAL_MOVE_POLARITY_FLIP_X) {
    manualMoveXMinus  = !digitalRead(DI_MANUAL_MOVE_X_PLUS);
    manualMoveXPlus = !digitalRead(DI_MANUAL_MOVE_X_MINUS);
  } else {
    manualMoveXPlus  = !digitalRead(DI_MANUAL_MOVE_X_PLUS);
    manualMoveXMinus = !digitalRead(DI_MANUAL_MOVE_X_MINUS);
  }
  if (MANUAL_MOVE_POLARITY_FLIP_Y) {
    manualMoveYMinus  = !digitalRead(DI_MANUAL_MOVE_Y_PLUS);
    manualMoveYPlus = !digitalRead(DI_MANUAL_MOVE_Y_MINUS);
  } else {
    manualMoveYPlus  = !digitalRead(DI_MANUAL_MOVE_Y_PLUS);
    manualMoveYMinus = !digitalRead(DI_MANUAL_MOVE_Y_MINUS);
  }
}

/******************************************************************************
    Reads travel limit inputs and updates global variables.
    by:     Greg

    Precondition:
    Postcondition:
      atLimitXMax: true = active
      atLimitXMin: true = active
      atLimitYMax: true = active
      atLimitYMin: true = active
        After each input, short dead time before the mode can be changed.

*/
void readTravelLimits() {
  atLimitXMax = !digitalRead(DI_LIMIT_SWITCH_X_MAX);  //pull-up resistor, switch pulls voltage down at limit
  atLimitXMin = !digitalRead(DI_LIMIT_SWITCH_X_MIN);
  atLimitYMax = !digitalRead(DI_LIMIT_SWITCH_Y_MAX);
  atLimitYMin = !digitalRead(DI_LIMIT_SWITCH_Y_MIN);
  if (!X_LIMIT_TRIP_SWITCH_N_O) {
    // normally closed and pulldowned down, logic low.  at limit would be high
    atLimitXMax = !atLimitXMax;
    atLimitXMin = !atLimitXMin;
  }
  if (!Y_LIMIT_TRIP_SWITCH_N_O) {
    // normally closed and pulldowned down, logic low.  at limit would be high
    atLimitYMax = !atLimitYMax;
    atLimitYMin = !atLimitYMin;
  }
  // TODO add debouncing
}

/******************************************************************************
    writes desired state to Bicolour LED
    for 3-wire bicolour LED with common cathode (negative side)
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      redPin:   arduino digital IO number for Red Anode
      greenPin: arduino digital IO number for Green Anode
      colour:   desired color: one of LED_COLOUR_XXX codes
      ledState: Current indicator state: true = on
    Precondition:
    Postcondition:
      LED state reflects status variables
*/
#define LED_DUTY_OFF          0       // out of 255
#define LED_DUTY_FULL         255     // out of 255
#define LED_DUTY_YELLOW_MIX_RED_SIDE  60      // out of 255
#define LED_DUTY_YELLOW_MIX_GREEN_SIDE  LED_DUTY_FULL
void writeBiColourLedPins(int redPin, int greenPin, int colour, bool ledState) {
  switch (colour) {
    case LED_COLOUR_RED:
      analogWrite(redPin, ledState ? LED_DUTY_FULL : LED_DUTY_OFF);
      analogWrite(greenPin, LED_DUTY_OFF);
      break;
    case LED_COLOUR_GREEN:
      analogWrite(redPin, LED_DUTY_OFF);
      analogWrite(greenPin, ledState ? LED_DUTY_FULL : LED_DUTY_OFF);
      break;
    case LED_COLOUR_YELLOW:
      analogWrite(redPin, ledState ? LED_DUTY_YELLOW_MIX_RED_SIDE : LED_DUTY_OFF);
      analogWrite(greenPin, ledState ? LED_DUTY_YELLOW_MIX_GREEN_SIDE : LED_DUTY_OFF);
      break;
    case LED_COLOUR_OFF:
      analogWrite(redPin, LED_DUTY_OFF);
      analogWrite(greenPin, LED_DUTY_OFF);
      break;
  }
}
