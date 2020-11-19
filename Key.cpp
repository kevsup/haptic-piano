#include <Key.h>

Key::Key(int pwm, int dir, int sensor, int fsr, double slope, double intercept) {
    state = PRESSING;
    prevState = PRESSING;
    pwmPin = pwm; // PWM output pin for motor 1
    dirPin = dir; // direction output pin for motor 1
    sensorPosPin = sensor; // input pin for MR sensor
    fsrPin = fsr;

    force = 0;
    updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
    rawPos = 0;         // current raw reading from MR sensor
    lastRawPos = 0;     // last raw reading from MR sensor
    lastLastRawPos = 0; // last last raw reading from MR sensor
    flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
    tempOffset = 0;
    rawDiff = 0;
    lastRawDiff = 0;
    rawOffset = 0;
    lastRawOffset = 0;
    flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
    flipped = false;
    m = slope;
    b = intercept;

    // Kinematics variables
    xh = 0;           // position of the handle [m]
    theta_s = 0; 

    //time
    t = 0;

    hardWallActivate = true;
}
