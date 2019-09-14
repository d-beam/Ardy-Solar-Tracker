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
