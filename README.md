Solar Tracker

Move a beewax melter in the right sun ascension using a DiSEqC 1.2 satellite motor.
Credits go to Graham Woan for the calculation part and controlling the motor.
original code can be found here: https://github.com/acrerd/Arduino-Diseqc-solar-tracker

"The arduino calculates the position of the Sun and sends tone controls to the drive via pin 8. 
!!! External electronics are required to inject the tone signals onto the drive power lines. !!!
Single-precision arithmetic limits the angular precision. A comparison with the JPL Horizons system show discrete errors of up to 1 minute in hour angle without the lookup tables. With the tables the errors are less (not checked how much less!)
22 kHz signal should be set to 650 mV p-p"

A few changes are made to make it work in 2025 (orginal code is from 2012). 
- tlookup to unsigned long as long cannot hold the last number
- changed from millis interrupt to just millis, 
  reading the gps data every second
  update the lcd every 10 seconds
  moving the drive every 10 min (i dont need the precise position as the antenna would)
- don't move (send command to move) the drive when it is already at its limits
- added a reverseMotor 'swicth' as i had to mount the motor upside down
- ADAFruit build error fix
  https://community.platformio.org/t/adafruit-gps-library-cannot-compile-because-softwareserial-is-missing/18703/7
  line 60 en 62 are changed 


 Known 'bugs'
 - counter (from 0 - 60) shown on the lCD (indicating when the motor will get its next command) doesn't clear on the display when reset to 0
   it shows 61, then reset to 1 it showns 11, 21, 31, 41, ... until it reaches 11.
   didnt fix that as that would require more memory and the UNO is almost at its limits and the counter itself works just fine
   
