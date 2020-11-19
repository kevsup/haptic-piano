#ifndef KEY_H
#define KEY_H

#include <Arduino.h>

//States for finite state machine
typedef enum {PRESSING, HARDWALL, INWALL, RETURNING, ABOVE} States;

class Key {

public:
    //Constructor
    Key(int pwm, int dir, int sensor, int fsr, double slope, double intercept);

    //state tracking
    States state;
    States prevState;

    double force;

    int pwmPin; // PWM output pin for motor 1
    int dirPin; // direction output pin for motor 1
    int sensorPosPin; // input pin for MR sensor
    int fsrPin; // input pin for FSR sensor

    // Position tracking variables
    int updatedPos;     // keeps track of the latest updated value of the MR sensor reading
    int rawPos;         // current raw reading from MR sensor
    int lastRawPos;     // last raw reading from MR sensor
    int lastLastRawPos; // last last raw reading from MR sensor
    int flipNumber;     // keeps track of the number of flips over the 180deg mark
    int tempOffset;
    int rawDiff;
    int lastRawDiff;
    int rawOffset;
    int lastRawOffset;
    int flipThresh;  // threshold to determine whether or not a flip over the 180 degree mark occurred
    boolean flipped;

    //calibration variables
    double m;
    double b;

    // Kinematics variables
    double xh;           // position of the handle [m]
    double theta_s;      // Angle of the sector pulley in deg
    double xh_prev;          // Distance of the handle at previous time step
    double dxh;              // Velocity of the handle
    double dxh_prev;
    double dxh_filt;

    //time tracker, for hard wall
    double t;

    //makes sure hardwall does not activate too often
    bool hardWallActivate; 

private:

};

#endif