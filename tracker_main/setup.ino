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
