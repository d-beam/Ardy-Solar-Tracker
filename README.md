# Ardy-Solar-Tracker
2-axis Solar Tracker for concentrators and mirrors, supporting alt-az and Fixed Equatorial mechanisms, safe position

This project is started in June 2018 by the Technology Group at the Temera Healing Biotope in Colos, Portugal.  It is a Arduino based solar tracking controller with the following software features:

# Features
* support 2 axes, configurable also for use with only 1 (X) axis.
* support mechanisms working with Alt-Az or fixed equatorial coordinants and actuators
* support configuration of absolute positioning (open-loop), using only limit switches as positional feedback.
** Configuration of real time/date and installation orientation (home position) required.
* for each axis, read analog signals from 2 photodiodes and a signal representing the difference of the two diode signals for closed-loop control
* switch between open-loop and closed-loop control based on the reliability of the sensor signal
* Usage Modes: Idle, Tracking on, safe position
* automatically disable tracking below a certain elevation, or outside a defined range in hour-angle
* sensor polarity inversion, configurable
* travel limit switches, configurable as normally open or normally closed
* inversion of the orientation of each push button pair for each axis, configurable
* 2 speeds of motor control, with configurable dead bands
* push buttons for manual movements
* Modularity to allow changing motor controllers or RTC modules

It is written in Arduino code, but also with some C concepts such as pointers,
functions with pointer arguements, etc.

# Suggested Improvements:
* split up source code into multiple files for logical modularity
* improve very rough position estimator, which integrates the movement command over time for a positional change.
* add configurable offset for differential sensor signals

# Future directions:
* As solar cooking controller: setting of cooking time and maximum temperature
* logging of solar intensity

Contact Daniel Müller <dbmueller@gmx.net> for more information about the project, including the related electronics hardware.
