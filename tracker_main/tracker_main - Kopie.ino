/******************************************************************************
    Solar Tracker: main Application for Arduino
    settings for Tamera FixedFocus Mirror
    July 7, 2018 15:15 2018

    Gregory Fung <gwfung@gmail.com>
    Free for use under Creative Commons license CC BY:
   https://creativecommons.org/licenses/
 *****************************************************************************/
#include "DualG2HighPowerMotorShield.h"

#include <Wire.h>
#include <RTClib.h>
#include <math.h>
#include <Helios.h>   // Heliostat / Sun location library

// Local configuration
// Tamera: 37° 43' 7.2001'' N, 8° 31' 14.1924'' W
#define LATITUDE      37.7186667f     // [-90..90]
#define LONGITUDE     -8.520609f      // [-180..180) 

// Application mode and settings --------------------
#define EQUATORIAL_AXES_MODE      true  // if false, output channels are Azimuth and elevation
// if false, output channels are X= tau hour angle and Y= delta declination
#define Y_AXIS_ACTIVE           true  // if false, all control functions except manual move is disabled for Y

#define OPEN_LOOP_POSITIONING_ENABLE  true  // if false, safe position and position initialization disabled.
// only tracking after rough manual alignment supported.

// Home position at min or max limit -----------
// true: homing at maximum position
// false: homing at mininum position
#define HOME_AT_MAX_X         false // morning, min tau position
#define HOME_AT_MAX_Y         true  // physically lowest position: summer, max delta

// Home position: --------------------
// The position of the system with minimum X and Y values, just before the limit switches are actuated.
// if EQUATORIAL_AXES_MODE, X is hour angle, Y is inclination
// if !EQUATORIAL_AXES_MODE, X is Azimuth, Y is elevation
#if EQUATORIAL_AXES_MODE
  #define HOME_POSITION_X           (-91.0) // °, morning position where low limit switches are located
  #define HOME_POSITION_Y           (23.5)  // ° where low limit switches are located
#else
  #define HOME_POSITION_X           (0.0)   // ° (north)
  #define HOME_POSITION_Y           (10.0)  // ° where low limit switches are located
#endif

// Safe position: --------------------
//Position for safe mode.  Must be within travel limits
#if EQUATORIAL_AXES_MODE
  #define SAFE_POSITION_X           (-90.0) // °
  #define SAFE_POSITION_Y           (23.0)  // °
#else
  #define SAFE_POSITION_X           (5.0)   // °
  #define SAFE_POSITION_Y           (11.0)  // °
#endif

// tracking active limitations: --------------------
// tracking will deactivate when the sun is calculated to be outside these ranges
#define SUN_ELEVATION_MIN_FOR_TRACKING      (10.0)  // °
#define SUN_HOUR_ANGLE_MIN_FOR_TRACKING     (-85.0) // °
#define SUN_HOUR_ANGLE_MAX_FOR_TRACKING     (85.0)  // °

// Manual move button polarity: --------------------
#define MANUAL_MOVE_POLARITY_FLIP_X       false   // true to invert control response, false otherwise
#define MANUAL_MOVE_POLARITY_FLIP_Y       true    // true to invert control response, false otherwise

// Limit switch action: --------------------
#define X_LIMIT_TRIP_SWITCH_N_O     true    // true: normally open switch, closes at limit
#define Y_LIMIT_TRIP_SWITCH_N_O     true    // true: normally open switch, closes at limit

// Sensor polarity: --------------------
#define SENSOR_POLARITY_FLIP_X      false   // true to invert control response, false otherwise
#define SENSOR_POLARITY_FLIP_Y      true    // true to invert control response, false otherwise

// Closed loop position sanity check: -------------------
#define CLOSED_LOOP_POSITIONING_SANITY_CHECK (false)      // true to enable comparison with calculated position

// Open loop position estimate: --------------------------
// Currently: Rectangular integration of motor movements commands
// set the steady-state movement speeds here, in °/s
#define MOTOR_SPEED_SLOW_PER_SEC_X      ((MOTOR_SPEED_FAST_PER_SEC_Y * POLOLU_SPEED_SLOW_Y) / POLOLU_SPEED_FAST_Y)    // speed ratiometric to Speed command?
#define MOTOR_SPEED_FAST_PER_SEC_X      0.54f   // °/s.  probably 10% accurate, needs further calibration
#define MOTOR_SPEED_SLOW_PER_SEC_Y      ((MOTOR_SPEED_FAST_PER_SEC_Y * POLOLU_SPEED_SLOW_Y) / POLOLU_SPEED_FAST_Y)    // °/s
#define MOTOR_SPEED_FAST_PER_SEC_Y      0.26f   // °/s.  probably 40% accurate, needs further calibration

// Control deadbands: --------------------------
// This reduces continual back and forth dithering due to slack in mechanical mechanism
//
// FAST_MOVE_DEADBAND: Within this much of the target location, stop using fast move speed
// CONTROL_DEADBAND: Within this much of the target location, stop moving
#define OPEN_LOOP_FAST_MOVE_DEADBAND_X    3   // °.
#define OPEN_LOOP_CONTROL_DEADBAND_X    0.3   // °
#define CLOSED_LOOP_FAST_MOVE_DEADBAND_X  500 // in raw diff ADC units (total range 2048, more than 1500 not recommended)
// Big setting to allow mechanism to decellerate and not overshoot
#define CLOSED_LOOP_CONTROL_DEADBAND_X    50    // in raw diff ADC units (total range 2048, more than 1500 not recommended)
#define CLOSED_LOOP_ABORT_LIMIT_X         25    // °, used for sanity check

#define OPEN_LOOP_FAST_MOVE_DEADBAND_Y    2   // °
#define OPEN_LOOP_CONTROL_DEADBAND_Y    0.3   // °
#define CLOSED_LOOP_FAST_MOVE_DEADBAND_Y  500 // in raw diff ADC units (total range 2048, more than 1500 not recommended)
#define CLOSED_LOOP_CONTROL_DEADBAND_Y    50  // in raw diff ADC units (total range 2048, more than 1500 not recommended)
#define CLOSED_LOOP_ABORT_LIMIT_Y         25  // °, used for sanity check


// movement delays: -------------------
// after stopping drive motors, how long to wait for mechanism to stop before proceeding
#define MOVE_STOP_FROM_FAST_DELAY (1000)    // ms
#define MOVE_STOP_FROM_SLOW_DELAY (1000)    // ms

// Closed loop tracking: -------------------
#define TRACKING_STABLE_COUNT       5       // count of loops before tracking stable is declared
                                            // used for position calibration

// Computation times: -------------------
// These constants may be used to integrate speed over time for position estimates
//
// Normal computation: happens every cycle of loop()
#define NORMAL_COMPUTATION_MS       (50)    // measured computational duration
// Extended computation: happens occassionally
#define EXTENDED_COMPUTATION_MS     (400)   // measured computational duration

// ******************************************************************************
// constants for internal operations, usually does not change between application
// ******************************************************************************

// Delay at the end of loop() before next cycle
#define UPDATE_DELAY_MS                 (250)
#define EXTENDED_COMPUTATION_COUNT      (60)

// Sensor limits, in raw 12-bit ADC units ------------
// ABS_MIN: below this level, sensor reading is not used.
//   lower to allow more tracking even if full cloudy,
//   may cause problems with background light when pointed away from sun
//   raise to use open loop calculated more often when cloudy
#define SENSOR_ABS_MIN      300       // 250 accepts signal levels when cloudy
                                      //~900 have been seen in full sun
// ABS_MAX: unrealistically strong sensor level
#define SENSOR_ABS_MAX      1800      // set >4096 to effectively disable checking
// REF: reference level of analog circuitry, which is also read by an ADC channel
#define SENSOR_REF_DEFAULT  2048      // half of full scale
#define SENSOR_REF_MAX      2248      // min resaonable ref value
#define SENSOR_REF_MIN      1848      // 

// Initialization ------------
// This many loops are ran before tracking will begin, to allow analog inputs to settle.
#define SENSOR_INIT_LOOP_COUNT    (16)

// Arduino GPIO numbers: analog inputs ----------------
#define AI_X1ABS            0
#define AI_X2ABS            1
#define AI_XDIFF            2
#define AI_XREF             3
#define AI_Y1ABS            4
#define AI_Y2ABS            5
#define AI_YDIFF            6
#define AI_YREF             7
#define AI_MOTOR_CURRENT_X  8
#define AI_MOTOR_CURRENT_Y  9

// Arduino GPIO numbers: digital ---------------
// 0, 1 used for serial comms
#define DO_MOTOR_PWM_X      2
#define DO_MOTOR_PWM_Y      3
#define DO_STATUS_LED_RED   4
#define DO_STATUS_LED_GREEN 5

#define DI_MODE_SAFE          43
#define DI_MODE_TRACK         41
#define DI_LIMIT_SWITCH_X_MAX 27
#define DI_LIMIT_SWITCH_X_MIN 29

//normal configration:
//#define DI_LIMIT_SWITCH_Y_MAX   23
//#define DI_LIMIT_SWITCH_Y_MIN   25
//compensating for wiring error on Tamera fixed-focus
#define DI_LIMIT_SWITCH_Y_MAX   25
#define DI_LIMIT_SWITCH_Y_MIN   23

#define DI_MANUAL_MOVE_X_PLUS   26
#define DI_MANUAL_MOVE_X_MINUS  28
#define DI_MANUAL_MOVE_Y_PLUS   24
#define DI_MANUAL_MOVE_Y_MINUS  22

// If motor PWM assignments match default config from
// pololu shield, timer1 can be used for PWM:  M1 9, M2 10
#define DO_MOTOR_DIR_X    42
#define DI_MOTOR_FAULT_X  40
#define DO_MOTOR_DIR_Y    46
#define DI_MOTOR_FAULT_Y  48

// unused, but referenced by motor driver
#define DO_MOTOR_SLEEP_X  47
#define DO_MOTOR_SLEEP_Y  45


// Serial Monitor debug output selection
// #define SERIAL_PLOTTER_FORMAT  // define for serial plotting csv format, disables labels
// #define SERIAL_DEBUG_RAW_SENSOR  // define to print raw sensor values
#define SERIAL_DEBUG_SUN_CALC   // define to enable

/******************************************************************************
   Global Variables
******************************************************************************/

// Using custom constructor here to specify non-default pin assignment
// Note the exact model of the sheild is in the type name and may need adjustment
// see https://github.com/pololu/dual-g2-high-power-motor-shield/
// TODO generic version DualG2HighPowerMotorShield?  current sensing gain info would not be encapsulated.
DualG2HighPowerMotorShield18v18 md(
  DO_MOTOR_SLEEP_X, DO_MOTOR_DIR_X,
  DO_MOTOR_PWM_X, DI_MOTOR_FAULT_X, AI_MOTOR_CURRENT_X,
  DO_MOTOR_SLEEP_Y, DO_MOTOR_DIR_Y,
  DO_MOTOR_PWM_Y, DI_MOTOR_FAULT_Y, AI_MOTOR_CURRENT_Y);
RTC_DS1307 rtc;
Helios sunPosition;

// actual date/time in UTC
DateTime actualTime;

// X axis---------------
int sensorX1RawAbs;
int sensorX2RawAbs;
int sensorXRawDiff;
int sensorXRawRef = SENSOR_REF_DEFAULT;
int sensorX1Level;
int sensorX2Level;
int sensorXDiff;
bool sensorLevelsOkX = false;
int motorSettingX = 0;
int motorCurrentX = 0;
bool atLimitXMax;
bool atLimitXMin;

bool closedLoopActiveX = false;
bool closedLoopFaultX = false;
bool trackingLockedX = false;
int trackingStableCounterX = 0;

double targetAzimuth;
float errorPositionX;     // position error for X in open loop mode
float actualPositionX;
double targetPositionX;
int errorClosedLoopX;     // control error for X, int because using raw ADC units


// Y axis--------------------
int sensorY1RawAbs;
int sensorY2RawAbs;
int sensorYRawDiff;
int sensorYRawRef = SENSOR_REF_DEFAULT;
int sensorY1Level;
int sensorY2Level;
int sensorYDiff;
bool sensorLevelsOkY = false;
int motorSettingY = 0;
int motorCurrentY = 0;
bool atLimitYMax;
bool atLimitYMin;

bool closedLoopActiveY = false;
bool closedLoopFaultY = false;
bool trackingLockedY = false;
int trackingStableCounterY = 0;

double targetElevation;
float errorPositionY;     // position error for Y in open loop mode
float actualPositionY;
double targetPositionY;
int errorClosedLoopY;     // control error for Y, int because using raw ADC units


// both axis--------------------
bool sensorInitDone = false;
bool generalFault = false;
bool userInputFault = false;
bool trackingRequested = false;   // user input
bool trackingActive = false;    // possible and active
bool safePositionCmd = false;
bool positionInitialized = false;
bool manualMoveXPlus = false;
bool manualMoveXMinus = false;
bool manualMoveYPlus = false;
bool manualMoveYMinus = false;
int slowUpdateCountdown = 0;

// LED
#define LED_COLOUR_OFF    0
#define LED_COLOUR_RED    1
#define LED_COLOUR_GREEN  2
#define LED_COLOUR_YELLOW 3

#define LED_BLINK_STEADY  0
#define LED_BLINK_SLOW    1
#define LED_BLINK_FAST    2

int ledColour = 0;
int ledBlink = 0;
bool ledActualState = false;
int ledActualColour = 0;
bool rtcOK = false;

const int ledBlinkInterval = 1000;      // ms, half of cycle
unsigned long previousLedMillis = 0;        // will store last time LED was updated


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

/******************************************************************************
   Real Time Clock
******************************************************************************/

/******************************************************************************
    intializes the real time clock
    by:     Knut

    Precondition:
    Postcondition:
      variables updated: year, month, day, hour, time, seconds
*/
void setupRealTimeClock() {
  rtc.begin();
  if (rtc.isrunning()) {
    rtcOK = true;
  } else {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to time of compilation
    rtc.adjust(DateTime(__DATE__, __TIME__) );
  }
  // use to setting RTC to compile time (note time zone!)
  // RTC needs to be set to UTC!
  rtc.adjust(DateTime(__DATE__, __TIME__) );
}

/******************************************************************************
    returns actual Time in UTC
    by:     Gregory Fung <gwfung@gmail.com>

*/
inline DateTime timeNow() {
  return rtc.now();
}

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

/******************************************************************************
   Sensing
******************************************************************************/

/******************************************************************************
    Reads the complimetary sensor signal inputs for axis X, assuming 12-bit ADC.
    by:     Gregory Fung <gwfung@gmail.com>

    Postcondition:
      sensorX1RawAbs: raw 12-bit ADC values [0..4095]
      sensorX2RawAbs: raw 12-bit ADC values [0..4095]
      sensorXRawDiff: raw 12-bit ADC values [0..4095]
      sensorXRawRef:  raw 12-bit ADC values [0..4095]
*/
#define SENSOR_XREF_IIR_DEPTH   (4) //bits, set >=2
void readTrackingSensorsX() {
  static int X1RawAccumulator = 4 * SENSOR_REF_DEFAULT;
  static int X2RawAccumulator = 4 * SENSOR_REF_DEFAULT;
  static int XRawDiffAccumulator = 2 * SENSOR_REF_DEFAULT;
  static unsigned int XRawRefAccumulator = 16 * SENSOR_REF_DEFAULT;

  // 1/4 IIR filter
  // 10-bit ADC multiplied by 4 to simulate 12-bit converter,
  // so this cancels the 1/4 division usually required to get the output from the accumulator
  X1RawAccumulator = X1RawAccumulator + analogRead(AI_X1ABS) - (sensorX1RawAbs >> 2);
  sensorX1RawAbs = X1RawAccumulator >> 2;
  X2RawAccumulator = X2RawAccumulator + analogRead(AI_X2ABS) - (sensorX2RawAbs >> 2);
  sensorX2RawAbs = X2RawAccumulator >> 2;

  // 1/2 IIR filter
  // 10-bit ADC multiplied by 4 to simulate 12-bit converter,
  // so this reduces the 1/4 division usually required to get the output from the accumulator to 1/2
  XRawDiffAccumulator = XRawDiffAccumulator + (analogRead(AI_XDIFF) << 1) - (sensorXRawDiff >> 1);
  sensorXRawDiff = XRawDiffAccumulator >> 1;

  // 1/16 IIR filter
  // 10-bit ADC multiplied by 4 to simulate 12-bit converter,
  // so this reduces the 1/16 division usually required to get the output from the accumulator to 1/4
  XRawRefAccumulator = XRawRefAccumulator + (analogRead(AI_XREF) >> (SENSOR_XREF_IIR_DEPTH - 2)) - (sensorXRawRef >> SENSOR_XREF_IIR_DEPTH);
  sensorXRawRef = XRawRefAccumulator >> SENSOR_XREF_IIR_DEPTH;
  /*
    Serial.print(", X1RawAcc ");
    Serial.print(X1RawAccumulator);
    Serial.print(", X2RawAcc ");
    Serial.print(X2RawAccumulator);
    Serial.print(", XRawDiffAcc ");
    Serial.print(XRawDiffAccumulator);
    Serial.print(", XRawRefAccu ");
    Serial.print(XRawRefAccumulator);
  */
}

/******************************************************************************
    Reads the complimetary sensor signal inputs for axis Y, assuming 12-bit ADC.
    by:     Gregory Fung <gwfung@gmail.com>

    Postcondition:
      sensorY1RawAbs: raw 12-bit ADC values [0..4095]
      sensorY2RawAbs: raw 12-bit ADC values [0..4095]
      sensorYRawDiff: raw 12-bit ADC values [0..4095]
      sensorYRawRef:  raw 12-bit ADC values [0..4095]
*/
#define SENSOR_YREF_IIR_DEPTH   (4) //bits, set >=2
void readTrackingSensorsY() {
  static int Y1RawAccumulator = 4 * SENSOR_REF_DEFAULT;;
  static int Y2RawAccumulator = 4 * SENSOR_REF_DEFAULT;;
  static int YRawDiffAccumulator = 2 * SENSOR_REF_DEFAULT;;
  static unsigned int YRawRefAccumulator = 16 * SENSOR_REF_DEFAULT;

  // 1/4 IIR filter
  // 10-bit ADC multiplied by 4 to simulate 12-bit converter,
  // so this cancels the 1/4 division usually required to get the output from the accumulator
  Y1RawAccumulator = Y1RawAccumulator + analogRead(AI_Y1ABS) - (sensorY1RawAbs >> 2);
  sensorY1RawAbs = Y1RawAccumulator >> 2;
  Y2RawAccumulator = Y2RawAccumulator + analogRead(AI_Y2ABS) - (sensorY2RawAbs >> 2);
  sensorY2RawAbs = Y2RawAccumulator >> 2;

  // 1/2 IIR filter
  // 10-bit ADC multiplied by 4 to simulate 12-bit converter,
  // so this reduces the 1/16 division usually required to get the output from the accumulator to 1/4
  YRawDiffAccumulator = YRawDiffAccumulator + (analogRead(AI_YDIFF) << 1) - (sensorYRawDiff >> 1);
  sensorYRawDiff = YRawDiffAccumulator >> 1;

  // 1/16 IIR filter
  // 10-bit ADC multiplied by 4 to simulate 12-bit converter,
  // so this reduces the 1/16 division usually required to get the output from the accumulator to 1/4
  YRawRefAccumulator = YRawRefAccumulator + (analogRead(AI_YREF) >> (SENSOR_XREF_IIR_DEPTH - 2)) - (sensorYRawRef >> SENSOR_XREF_IIR_DEPTH);
  sensorYRawRef = (YRawRefAccumulator >> SENSOR_XREF_IIR_DEPTH);
}

/******************************************************************************
    Interprets raw ADC values and performs sanity checks.
    by:     Gregory Fung <gwfung@gmail.com>

    Precondition:
      SENSOR_REF_MIN: min reasonable value for sensor reference
      SENSOR_REF_MAX: max reasonable value for sensor reference
      p_rawRef: pointer to raw 12-bit ADC value, reasonable [SENSOR_REF_MIN..SENSOR_REF_MAX]
      rawRefDefault: reference level for input signal
      p_rawAbs1: pointer to raw 12-bit ADC values [0..4095]
      p_rawAbs2: pointer to raw 12-bit ADC values [0..4095]
      p_rawDiff: pointer to raw 12-bit ADC values [0..4095]
      p_level1: pointer to raw 12-bit ADC units, positive and reference level subtracted.  [0..4095]
      p_level2: pointer to raw 12-bit ADC units, positive and reference level subtracted.  [0..4095]
      p_diff: pointer to raw 12-bit ADC units, reference level subtracted.  [-2048..2048]
      abs_min: absolute min level for input signal
      abs_max: absolute max level for input signal
    Postcondition:
      p_level1: output to addressed int: positive and reference level subtracted.  [0..4095]
      p_level2: output to addressed int: positive and reference level subtracted.  [0..4095]
      p_diff:   output to addressed int: reference level subtracted.  [-2048..2048]
    Returns:  true if sensors OK
*/
bool CheckSensorLevels(int* p_rawRef, int rawRefDefault, int* p_rawAbs1, int* p_rawAbs2, int* p_rawDiff,
                       int* p_level1, int* p_level2, int* p_diff,
                       int abs_min, int abs_max) {
  bool xRefOK;
  int xRef;
  // check reference reading
  xRefOK = ((*p_rawRef >= SENSOR_REF_MIN) && (*p_rawRef <= SENSOR_REF_MAX));
  if (xRefOK) {
    xRef = *p_rawRef;
  } else {
    xRef = rawRefDefault;
  }

  //subtract reference off read values, clamp negative values to 0
  *p_level1 = *p_rawAbs1 - xRef;
  if (*p_level1 < 0) {
    *p_level1 = 0;
  }
  *p_level2 = *p_rawAbs2 - xRef;
  if (*p_level2 < 0) {
    *p_level2 = 0;
  }
  *p_diff = *p_rawDiff - xRef;

  //check against min/max.  Not OK if both sensors have weak signals, or either one is above max
  return (((*p_level1 >= abs_min) || (*p_level2 >= abs_min)) &&
          (*p_level1 <= abs_max) &&
          (*p_level2 <= abs_max));

  // TODO: sanity check polarity of diff signal, based on the 2 absolute signals
}


/******************************************************************************
   Sun location
******************************************************************************/
#define DEG_TO_RAD(x)  ((x)*pi/180.0)
#define RAD_TO_DEG(x)  ((x)*180/pi)

/******************************************************************************
    Calculates the expected location of the sun, based on the configured location and actual time.
    by:     Christian Oekermann <christian@oekermann.com>

    Parameters:
      time: in UTC
      sunPosition: Helios object
      longtitude: in degrees
      latitude: in degrees
      p_azi: pointer to double: Azimuth
      p_elev: pointer to double: Elevation
    Postcondition:
      location pointed to by p_azi: azimuth in degrees [0..360)
      location pointed to by p_elev: elevation in degrees [-90..90)
*/
void calculateSunLocation(DateTime time, Helios sunPosition,
                          double longtitude, double latitude, double* p_azi, double* p_elev) {
  sunPosition.calcSunPos(time.year(), time.month(), time.day(),
                         time.hour(), time.minute(), time.second(),
                         longtitude, latitude);
  *p_azi = sunPosition.dAzimuth;
  *p_elev = sunPosition.dElevation;
}

/******************************************************************************
    Calculates the expected location pair from alt azimuth to stable Equatorial coordinate system
    ref: https://de.wikipedia.org/wiki/Astronomische_Koordinatensysteme#Horizontale_(a,_h)_%E2%86%92_kartesische_Koordinaten_%E2%86%92_ruhende_%C3%A4quatoriale_Koordinaten_(%CF%84,_%CE%B4)

    Status:   unit test of this gave incorrect results!

    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      latitude: latitude of observer location
      p_delta_DEG: pointer to output double: Declination in degrees [-90..90°)
    output:
      targetPositionX: equatorial rotation in degrees [0..360)
      targetPositionY: omega in degrees [-90..90)
*/
void convertToStableEquatorialAxes_DEAD_CODE(double latitude, double azimuth, double elevation, 
  double * p_tau_DEG, double * p_delta_DEG) {
  /*
    ϕ \phi  = geographische Breite (latitude)
    a  a  = Azimut
    h  h  = Höhenwinkel (elevation)
    τ  \tau   = Stundenwinkel (hour angle)
    δ  \delta   = Deklination (declination)

    δ = arcsin ⁡ ( sin ⁡ ϕ ⋅ sin ⁡ h − cos ⁡ ϕ ⋅ cos ⁡ h ⋅ cos ⁡ a )
    τ = arctan ⁡ ( sin ⁡ a / (sin ⁡ ϕ ⋅ cos ⁡ a + cos ⁡ ϕ ⋅ tan ⁡ h ))
  */

  double lat_RAD = DEG_TO_RAD(latitude);
  double azi_RAD = DEG_TO_RAD(azimuth);
  double elev_RAD = DEG_TO_RAD(elevation);

  double sin_phi = sin(lat_RAD);
  double cos_phi = cos(lat_RAD);
  double sin_a = sin(azi_RAD);
  double cos_a = cos(azi_RAD);
  double sin_h = sin(elev_RAD);
  double cos_h = cos(elev_RAD);
  double tan_h = tan(elev_RAD); //TODO handle + or - 90° case?

  double sin_delta = sin_phi * sin_h - cos_phi * cos_h * cos_a;
  double delta_RAD = asin(sin_delta);
  *p_delta_DEG = RAD_TO_DEG(delta_RAD);

  double tau_denominator = sin_phi * cos_a + cos_phi * tan_h;
  double tan_tau = sin_a / tau_denominator;
  double tau_RAD = atan(tan_tau);
  //atran only returns results in quadrants I and IV.   Adjust for results in quadrant II and III
  if (tau_denominator < 0) {      //left half plane
    //quadrant II: correct from quadrant IV; quadrant III: correct from quadrant I
    tau_RAD += pi;        //same correction for both
  }
  *p_tau_DEG = RAD_TO_DEG(tau_RAD);

  /* math debug
    Serial.print(",| lat_RAD ");
    Serial.print(lat_RAD);
    Serial.print(", azi_RAD ");
    Serial.print(azi_RAD);
    Serial.print(", elev_RAD ");
    Serial.print(elev_RAD);

    Serial.print(", sin_phi ");
    Serial.print(sin_phi);
    Serial.print(", cos_phi ");
    Serial.print(cos_phi);
  */
  Serial.print(",| sin_a ");
  Serial.print(sin_a);
  Serial.print(", cos_a ");
  Serial.print(cos_a);
  Serial.print(", sin_h ");
  Serial.print(sin_h);
  Serial.print(", cos_h ");
  Serial.print(cos_h);
  Serial.print(", tan_h ");
  Serial.print(tan_h);
  Serial.print(",| tau_denominator ");
  Serial.print(tau_denominator);
  Serial.print(", tan_tau ");
  Serial.print(tan_tau);
  Serial.print(", sin_delta ");
  Serial.print(sin_delta);


  Serial.print(",| Equa: tau ");
  Serial.print(*p_tau_DEG);
  Serial.print("deg,| delta ");
  Serial.print(*p_delta_DEG);
  Serial.print("deg");
}

/******************************************************************************
    Calculates the expected location pair from alt azimuth to stable Equatorial coordinate system
    ref: http://star-www.st-and.ac.uk/~fv/webnotes/chapter7.htm
    (not implementing the step from hour angle to RA)
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      latitude: latitude of observer location
      azimuth: azimuth in degrees [0..360)
      elevation: elevation in degrees [-90..90)
      p_tau_DEG: pointer to output double: Stundenwinkel (hour angle) in degrees [0..360°)
      p_delta_DEG: pointer to output double: Declination in degrees [-90..90°)
    output:
      targetPositionX: equatorial rotation in degrees [-180..180)
      targetPositionY: omega in degrees [-90..90)
*/
void convertToStableEquatorialAxes(double latitude, double azimuth, double elevation,
                                   double * p_tau_DEG, double * p_delta_DEG) {

  /*
    azimuth A
    altitude a (elevation)
    latitude φ
    Local Hour Angle H (also known as τ tau)
    declination δ
    Given φ phi, a and A, what are H (τ) and δ delta?

    sin(δ) = sin(a)sin(φ) + cos(a) cos(φ) cos(A)
    sin(H) = - sin(A) cos(a) / cos(δ)
    cos(H) = { sin(a) - sin(δ) sin(φ) } / cos(δ) cos(φ)
    use both cos and sin H to map the angle to the correct quadrant.
  */

  double lat_RAD = DEG_TO_RAD(latitude);
  double elev_RAD = DEG_TO_RAD(elevation);
  double azi_RAD = DEG_TO_RAD(azimuth);

  double sin_a = sin(elev_RAD);
  double cos_a = cos(elev_RAD);
  double sin_phi = sin(lat_RAD);
  double cos_phi = cos(lat_RAD);
  double sin_A = sin(azi_RAD);
  double cos_A = cos(azi_RAD);

  double sin_delta = sin_a * sin_phi + cos_a * cos_phi * cos_A;
  double delta_RAD = asin(sin_delta);     //output range [-90..90°], quad I and IV
  double cos_delta = cos(delta_RAD);
  *p_delta_DEG = RAD_TO_DEG(delta_RAD);
  *p_delta_DEG = correctArcsineQuadrant(cos_delta, *p_delta_DEG);

  double sin_H = -1 * sin_A * cos_a / cos_delta;
  double cos_H_numerator = (sin_a - sin_delta * sin_phi);
  double cos_H = cos_H_numerator / (cos_delta * cos_phi);
  *p_tau_DEG = RAD_TO_DEG(asin(sin_H));   //output range [-90..90°], quad I and IV
  *p_tau_DEG = correctArcsineQuadrant(cos_H, *p_tau_DEG);

  /* math debug
    Serial.print(",| sin_a ");
    Serial.print(sin_a);
    Serial.print(", cos_a ");
    Serial.print(cos_a);
    Serial.print(", sin_phi ");
    Serial.print(sin_phi);
    Serial.print(", cos_phi ");
    Serial.print(cos_phi);
    Serial.print(", sin_A ");
    Serial.print(sin_A);
    Serial.print(", cos_A ");
    Serial.print(cos_A);

    Serial.print(",| sin_delta ");
    Serial.print(sin_delta);
    Serial.print(", cos_delta ");
    Serial.print(cos_delta);
    Serial.print(", delta_RAD ");
    Serial.print(delta_RAD);
    Serial.print(", sin_H ");
    Serial.print(sin_H);
    Serial.print(", cos_H_num ");
    Serial.print(cos_H_numerator);
    Serial.print(", cos_H ");
    Serial.print(cos_H);
  */

#ifdef SERIAL_DEBUG_SUN_CALC
  Serial.print(",| in fixed Equatorial: delta ");
  Serial.print(*p_delta_DEG);
  Serial.print("deg, H tau ");
  Serial.print(*p_tau_DEG);
  Serial.print("deg");
#endif
}

double correctArcsineQuadrant(double cos_angle, double angle) {
  double result;
  // Arcsin gives unclear result for left or right half plane.
  // Determine from cos if we should be in the left half plane.
  result = angle;
  if (cos_angle < 0) {
    //mirror quad I to quad II, mirror quad IV to quad III
    result = 180.0 - angle;     // angle complement works perfectly.  eg: -45°: 180 - -45 = 225°
  }
  if (result < -180) {        // should never be, but this code doesn't hurt
    result += 360.0;
  } else if (result > 180) {
    result -= 360.0;
  }
  return result;
}

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

/******************************************************************************
   main loops for testing
******************************************************************************/

/******************************************************************************
   sun location unit test
   by:      Gregory Fung <gwfung@gmail.com>
*/
void sunCalcTest() {
  actualTime = timeNow();

  /* RTC */
  Serial.print("Time ");
  writeDebugOutputDateTime(actualTime);
  Serial.println("");
  for (int h = 0; h < 24; h++) {
    for (int m = 0; m < 60; m += 20) {
      DateTime testTime = DateTime(actualTime.year(), actualTime.month(), actualTime.day(),
                                   h, m, actualTime.second());
      Serial.print("");
      writeDebugOutputTime(testTime);
      sunPosition.calcSunPos(testTime.year(), testTime.month(), testTime.day(),
                             testTime.hour(), testTime.minute(), testTime.second(),
                             LONGITUDE, LATITUDE);
      targetPositionX = sunPosition.dAzimuth;
      targetPositionY = sunPosition.dElevation;

      Serial.print(", Az:");
      Serial.print(sunPosition.dAzimuth);
      Serial.print(", Elev:");
      Serial.print(sunPosition.dElevation);

      convertToStableEquatorialAxes(LATITUDE, sunPosition.dAzimuth, sunPosition.dElevation,
                                    &targetPositionX, &targetPositionY);
      Serial.print(", targetPosition X");
      Serial.print(targetPositionX);
      Serial.print(" Y");
      Serial.print(targetPositionY);
      Serial.println();
    }
  }
  Serial.println("---");
  for (int m = 1; m <= 12; m++) {
    DateTime testTime = DateTime(actualTime.year(), m, actualTime.day(),
                                 actualTime.hour(), actualTime.minute(), actualTime.second());
    Serial.print("");
    writeDebugOutputDateTime(testTime);
    sunPosition.calcSunPos(testTime.year(), testTime.month(), testTime.day(),
                           testTime.hour(), testTime.minute(), testTime.second(),
                           LONGITUDE, LATITUDE);
    targetPositionX = sunPosition.dAzimuth;
    targetPositionY = sunPosition.dElevation;

    Serial.print(", Az:");
    Serial.print(sunPosition.dAzimuth);
    Serial.print(", Elev:");
    Serial.print(sunPosition.dElevation);
    convertToStableEquatorialAxes(LATITUDE, sunPosition.dAzimuth, sunPosition.dElevation,
                                  &targetPositionX, &targetPositionY);

    Serial.print(", targetPosition X");
    Serial.print(targetPositionX);
    Serial.print(" Y");
    Serial.print(targetPositionY);
    Serial.println();
  }
  Serial.println("*****");
  delay(10000);
}

/******************************************************************************
   LED tester
   by:      Gregory Fung <gwfung@gmail.com>
*/
#define TEST_VECTOR_SIZE 7
int test_LED_COLOUR_vector[TEST_VECTOR_SIZE] = {LED_COLOUR_RED, LED_COLOUR_RED, LED_COLOUR_GREEN, LED_COLOUR_GREEN,
                                                LED_COLOUR_YELLOW, LED_COLOUR_YELLOW, LED_COLOUR_OFF
                                               };
bool test_blink_vector[TEST_VECTOR_SIZE] = {2, 0, 1, 0, 1, 0, 0};
void test_led() {
  static int counter = 10;
  static int i = 0;
  if (counter++ >= 10) {
    counter = 0;
    //ledColour = test_LED_COLOUR_vector[i];
    ledColour = LED_COLOUR_YELLOW;
    ledBlink = test_blink_vector[i];
    Serial.println(i);        //debug
    i++;
    if (i <= TEST_VECTOR_SIZE) {
      i = 0;
    }
  }
  ledStateUpdate();

  delay(200);
}

/******************************************************************************
   Sets up the tracker on boot up.
   by:      Gregory Fung <gwfung@gmail.com>
*/
void setup() {
  arduino_init();
  writeDebugOutputDateTime(actualTime);
  Serial.println("");

  // for testing
  //sunCalcTest();

  // run main loop a few times to stabilize analog input filtering, before allowing any active system modes
  for (int i = 0; i < SENSOR_INIT_LOOP_COUNT; i++) {
    loop();
  }
  sensorInitDone = true;
  Serial.println("System Init complete");
}

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
