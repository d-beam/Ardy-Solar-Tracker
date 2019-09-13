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
