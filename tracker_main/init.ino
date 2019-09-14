/******************************************************************************
   Arduino system
******************************************************************************/

/******************************************************************************
    Sets up Arduino system related items.
    by:     Knut

    Precondition:
    Postcondition:
      Arduino and accesories are initialized from a HW driver perspective

*/
void arduino_init() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println("arduino_init start");

  analogReference(DEFAULT);   //5V board reference

  // pololu Motor driver
  md.init();
  md.calibrateCurrentOffsets();

  //Wire for I2C
  Wire.begin();
  setupRealTimeClock();
  actualTime = timeNow();

  // setup up inputs and outputs
  pinMode(DI_MODE_SAFE, INPUT_PULLUP);
  pinMode(DI_MODE_TRACK, INPUT_PULLUP);
  pinMode(DI_LIMIT_SWITCH_X_MAX, INPUT_PULLUP);
  pinMode(DI_LIMIT_SWITCH_X_MIN, INPUT_PULLUP);
  pinMode(DI_LIMIT_SWITCH_Y_MAX, INPUT_PULLUP);
  pinMode(DI_LIMIT_SWITCH_Y_MIN, INPUT_PULLUP);
  pinMode(DI_MANUAL_MOVE_X_PLUS, INPUT_PULLUP);
  pinMode(DI_MANUAL_MOVE_X_MINUS, INPUT_PULLUP);
  pinMode(DI_MANUAL_MOVE_Y_PLUS, INPUT_PULLUP);
  pinMode(DI_MANUAL_MOVE_Y_MINUS, INPUT_PULLUP);

  pinMode(DO_STATUS_LED_RED, OUTPUT);
  pinMode(DO_STATUS_LED_GREEN, OUTPUT);

  /* Motor controller IOs done by md.init   */

  // setup up limit switch interrupts
  // TODO?

}
