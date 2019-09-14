/******************************************************************************
   Motor Control
******************************************************************************/

#define MOTOR_DIR_PLUS      (true)
#define MOTOR_DIR_MINUS     (false)
#define MOTOR_SPEED_FAST    (2)
#define MOTOR_SPEED_SLOW    (1)
#define MOTOR_SPEED_OFF     (0)

#define MOTOR_CURRENT_LIMIT_X   (8000)    // mA
#define MOTOR_CURRENT_LIMIT_Y   (18000)   // mA

/* development model
  #define POLOLU_SPEED_FAST_X   (400) // pololu max 400
  #define POLOLU_SPEED_SLOW_X   (25)  // pololu max 400
  #define POLOLU_SPEED_FAST_Y   (400) // pololu max 400
  #define POLOLU_SPEED_SLOW_Y   (25)  // pololu max 400
*/
/* Tamera FixFocus */
// pololu maxinum 400
#define POLOLU_SPEED_FAST_X   (400) //
#define POLOLU_SPEED_SLOW_X   (100) // 
#define POLOLU_SPEED_FAST_Y   (300) // fast enough and reduces current spikes
#define POLOLU_SPEED_SLOW_Y   (250) //

/******************************************************************************
    Checks for motor faults and handles as required.  Based on demo from pololu.
    by:     Gregory Fung <gwfung@gmail.com>

    Returns:    false on motor fault input or overcurrent, true otherwise.
    Postcondition:
      Motor drivers disabled
*/
bool areMotorsOK() {
  bool ok = true;
  motorCurrentX = md.getM1CurrentMilliamps();
  //motorCurrentX = 0;
  if (motorCurrentX > MOTOR_CURRENT_LIMIT_X || md.getM1Fault()) {
    md.disableM1Driver();
    delay(1);
    ok = false;
    if (md.getM1Fault()) {
      Serial.println("Motor X (M1) fault");
    } else {
      Serial.println("Motor X (M1) overcurrent");
    }
  }
  motorCurrentY = md.getM2CurrentMilliamps();
  //motorCurrentY = 0;
  if (motorCurrentY > MOTOR_CURRENT_LIMIT_Y || md.getM2Fault()) {
    md.disableM2Driver();
    delay(1);
    ok = false;
    if (md.getM1Fault()) {
      Serial.println("Motor Y (M2) fault");
    } else {
      Serial.println("Motor Y (M2) overcurrent");
    }
  }
  return ok;
}

/******************************************************************************
    Starts movement of the tracker in the X axis, in the given direction at the given speed.
    if movement was already active in the opposite direction, movement shall be stopped.
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      direction: true in positive direction, false in negative direction
      speed: 0: stop, 1 slow, 2 fast
    Precondition:
    Postcondition:
      targetPositionX: azimuth in degrees [0..360)
      targetPositionY: elevation in degrees [-90..90)
*/
void driveX(bool direction, int speed) {
  int pololuSpeed = 0;
  static bool lastDirection = false;

  // if driver was off before
  if ((speed > 0) && (motorSettingX == 0)) {
    md.enableM1Driver();
    delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
  }

  // stop first if changing directions
  if ((speed > 0) && (direction != lastDirection)) {
    speed = 0;
  }

  // TODO consider speed ramp up and ramp down?
  if (speed >= 2) {
    pololuSpeed = POLOLU_SPEED_FAST_X;
  } else if (speed == 1) {
    pololuSpeed = POLOLU_SPEED_SLOW_X;
  } else {
    pololuSpeed = 0;
  }

  lastDirection = direction;  //update last motor setting memory
  motorSettingX = speed;
  if (!direction) {
    pololuSpeed = -1 * pololuSpeed;
    motorSettingX = -1 * speed;
  }

  // drive motor
  md.setM1Speed(pololuSpeed);

  if (speed == 0) {
    md.disableM1Driver();
  }
}

/******************************************************************************
    Starts movement of the tracker in the Y axis, in the given direction at the given speed.
    if movement was already active in the opposite direction, movement shall be stopped.
    by:     ?

    Parameters:
      direction: true in positive direction, false in negative direction
      speed: 0: stop, 1 slow, 2 fast
    Precondition:
    Postcondition:
*/
void driveY(bool direction, int speed) {
  int pololuSpeed = 0;
  static bool lastDirection = false;

  // if driver was off before
  if ((speed > 0) && (motorSettingY == 0)) {
    md.enableM2Driver();
    delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
  }

  // stop first if changing directions
  if ((speed > 0) && (direction != lastDirection)) {
    speed = 0;
  }

  // TODO consider speed ramp up and ramp down?
  if (speed >= 2) {
    pololuSpeed = POLOLU_SPEED_FAST_Y;
  } else if (speed == 1) {
    pololuSpeed = POLOLU_SPEED_SLOW_Y;
  } else {
    pololuSpeed = 0;
  }

  lastDirection = direction;  //update last motor setting memory
  motorSettingY = speed;
  if (!direction) {
    pololuSpeed = -1 * pololuSpeed;
    motorSettingY = -1 * speed;
  }
  // drive motor
  md.setM2Speed(pololuSpeed);
  if (speed == 0) {
    md.disableM2Driver();
  }
}
