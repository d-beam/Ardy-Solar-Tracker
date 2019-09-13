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
