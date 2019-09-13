/******************************************************************************
    Solar Tracker: Application for Arduino
    settings for Tamera FixedFocus Mirror
    July 7, 2018 15:15 2018

    Update on 13.9.2019 
    - split into diffrent files 
     
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
