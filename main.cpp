#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

//For switching between different modes
#define PIANO
//#define HARPSICHORD

// Includes
#include <math.h>
#include <Key.h>

//Declare Key objects
Key midKey = Key(5, 8, A2, A3, 0.0134, -5.469);
Key leftKey = Key(3, 2, A0, A1, 0.0103, -4.3046);
Key rightKey = Key(6, 7, A4, A5, -0.0125, 8.5005);

//used to calibrate the Hapkit once during the loop; this proved more effective
//than calibrating during setup()
int clbCt = 0;  

//For use in controlling serial prints, so that it does not print
//too often and cause instabilities due to printing delay
int count = 50;

//Function prototype for setPwmFrequency
void setPwmFrequency(int pin, int divisor);

//Sets up keys and the corresponding pins
void setupKey(Key& key) {
  setPwmFrequency(key.pwmPin,1); 
  pinMode(key.sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(key.fsrPin, INPUT);       // set FSR sensor pin to be an input
  pinMode(key.pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(key.dirPin, OUTPUT);  // dir pin for motor A
  analogWrite(key.pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(key.dirPin, LOW);  // set direction
  key.lastLastRawPos = analogRead(key.sensorPosPin);
  key.lastRawPos = analogRead(key.sensorPosPin);
}

//Calibrates the keys to 0
void checkCalibrate() {
  double leftTot = 0;
  double midTot = 0;
  double rightTot = 0;
  int numMeasurements = 10;
  for (int i = 0; i < numMeasurements; i++) {
    leftTot += -leftKey.m * leftKey.updatedPos;
    midTot += -midKey.m * midKey.updatedPos;
    rightTot += -rightKey.m * rightKey.updatedPos;
  }
  leftKey.b = leftTot / numMeasurements;
  midKey.b = midTot / numMeasurements;
  rightKey.b = rightTot / numMeasurements;
}

void setup() 
{
  // Set up serial communication
  Serial.begin(9600);
  setupKey(midKey);
  setupKey(leftKey);
  setupKey(rightKey);
}

//function prototype for function that computes PWM output 
//given force, output pin, and direction pin
void computeForce(double force, int outpin, int dir);

//Compute position in counts for a key
void computePosInCounts(Key& key) {
  key.rawPos = analogRead(key.sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  key.rawDiff = key.rawPos - key.lastRawPos;          //difference btwn current raw position and last raw position
  key.lastRawDiff = key.rawPos - key.lastLastRawPos;  //difference btwn current raw position and last last raw position
  key.rawOffset = abs(key.rawDiff);
  key.lastRawOffset = abs(key.lastRawDiff);
  
  // Update position record-keeping vairables
  key.lastLastRawPos = key.lastRawPos;
  key.lastRawPos = key.rawPos;
  
  // Keep track of flips over 180 degrees
  if((key.lastRawOffset > key.flipThresh) && (!key.flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(key.lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      key.flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      key.flipNumber++;              // ccw rotation
    }
    if(key.rawOffset > key.flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      key.updatedPos = key.rawPos + key.flipNumber*key.rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      key.tempOffset = key.rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      key.updatedPos = key.rawPos + key.flipNumber*key.lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      key.tempOffset = key.lastRawOffset;
    }
    key.flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    key.updatedPos = key.rawPos + key.flipNumber*key.tempOffset; // need to update pos based on what most recent offset is 
    key.flipped = false;
  }
}

//Computes position in meters for a key
void computePosInMeters(Key& key, double rh) {
  // Step B.6: Compute the angle of the sector pulley (ts) in degrees based on updatedPos 
  double ts = key.m * key.updatedPos + key.b;
  // Step B.7: Compute the position of the handle (in meters) based on ts (in radians)
  key.xh_prev = key.xh;
  key.xh = rh*ts*3.14159/180;
  // Calculate velocity with loop time estimation
  key.dxh_prev = key.dxh;
  key.dxh = (double)(key.xh - key.xh_prev) / 0.001;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  key.dxh_filt = .9*key.dxh + 0.1*key.dxh_prev; 
}

//Renders the virtual piano
void pianoRender(Key& key, double kp, double kd, double bottomThresh, double kWall, double returnForce) {
  switch (key.state) {
    case PRESSING:
      key.force = kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
      if (key.xh > bottomThresh) {
        key.force += kWall*(bottomThresh - key.xh);
        key.prevState = PRESSING;
        key.state = HARDWALL;
      } else if (key.dxh_filt <= 0) {
        key.prevState = PRESSING;
        key.state = RETURNING;
      }
      break;
    case HARDWALL:
      if (key.hardWallActivate) {
        if (key.dxh_filt > 0) {
          //vibration upon wall contact
          int b = 10;
          key.force = kWall*(bottomThresh - key.xh) - b*key.dxh_filt + kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
        } else {
          key.force = kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
        }
      }
      key.t += 0.005; //rough estimate of loop time increment
      if (key.t > 0.03) {
        key.t = 0;
        key.hardWallActivate = false;
        key.prevState = HARDWALL;
        key.state = INWALL;
      }
      break;
    case INWALL:
      if (key.xh < bottomThresh) {
        key.prevState = INWALL;
        key.state = RETURNING;
      } else {
        key.force = kWall*(bottomThresh - key.xh) + returnForce;
      }    
      break;
    case RETURNING:
      if (key.xh > bottomThresh) {
        key.prevState = RETURNING;
        key.state = INWALL;
      } else if (key.dxh_filt > 0.1 && key.xh < bottomThresh/2) {
        key.prevState = RETURNING;
        key.state = PRESSING; 
      } else if (key.xh <= 0) {
        key.prevState = RETURNING;
        key.state = ABOVE;
      } else { 
        if (key.xh < bottomThresh/3) {
          key.force = returnForce/3;
        } else if (key.xh < bottomThresh/5) {
          key.force = (returnForce/3 - returnForce/5)/(bottomThresh/3 - 
                          bottomThresh/5)*(key.xh - bottomThresh/3) + returnForce/3;
        } else {
          key.force = returnForce;
        }
      }
      break;
    case ABOVE:
      key.hardWallActivate = true;
      if (key.xh < 0) {
        key.force = kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
      } else if (key.xh > 0) {
        key.prevState = ABOVE;
        key.state = RETURNING;
      }
      break;
    default:
      //shouldn't be stateless
      Serial.println("Stateless");
  }
  
}

//Renders the virtual harpsichord
void harpsichordRender(Key& key, double kp, double kd, double bottomThresh, double kWall, double returnForce) {
  switch (key.state) {
    case PRESSING:
      key.force = kp*(0 - key.xh) + kd*(0 - key.dxh_filt);

      if (key.xh > bottomThresh) {
        key.force += kWall*(bottomThresh - key.xh);
        key.prevState = PRESSING;
        key.state = HARDWALL;
      } else if (key.xh > bottomThresh*3/5 && key.xh < bottomThresh*3/4) {
        double kString = 600;
        key.force += kString * (bottomThresh*3/5 - key.xh);      
      } else if (key.dxh_filt <= 0) {
        key.prevState = PRESSING;
        key.state = RETURNING;
      }     
      break;
    case HARDWALL:
      if (key.hardWallActivate) {
        if (key.dxh_filt > 0) {
          //vibration upon wall contact
          int b = 10;
          key.force = kWall*(bottomThresh - key.xh) - b*key.dxh_filt + kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
        } else {
          key.force = kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
        }
      }
      key.t += 0.005; //rough estimate of loop time increment
      if (key.t > 0.03) {
        key.t = 0;
        key.hardWallActivate = false;
        key.prevState = HARDWALL;
        key.state = INWALL;
      }
      break;
    case INWALL:
      if (key.xh < bottomThresh) {
        key.prevState = INWALL;
        key.state = RETURNING;
      } else {
        key.force = kWall*(bottomThresh - key.xh) + returnForce;
      }    
      break;
    case RETURNING:
      if (key.xh > bottomThresh) {
        key.prevState = RETURNING;
        key.state = INWALL;
      } else if (key.dxh_filt > 0.1 && key.xh < bottomThresh/2) {
        key.prevState = RETURNING;
        key.state = PRESSING;       
      } else if (key.xh <= 0) {
        key.prevState = RETURNING;
        key.state = ABOVE;
      } else { 
        if (key.xh < bottomThresh/3) {
          key.force = returnForce/3;
        } else if (key.xh < bottomThresh/5) {
          key.force = (returnForce/3 - returnForce/5)/(bottomThresh/3 - 
                          bottomThresh/5)*(key.xh - bottomThresh/3) + returnForce/3;
        } else {
          key.force = returnForce;
        }
      }
      break;
    case ABOVE:
      key.hardWallActivate = true;
      if (key.xh < 0) {
        key.force = kp*(0 - key.xh) + kd*(0 - key.dxh_filt);
      } else if (key.xh > 0) {
        key.prevState = ABOVE;
        key.state = RETURNING;
      }
      break;
    default:
      //shouldn't be stateless
      Serial.println("Stateless");
  }
  
}


void loop()
{
  //calibrate once during loop: more effective than calibration during setup()
  if (clbCt == 50) {
    checkCalibrate();
    clbCt++;
  } else if (clbCt < 50) {
    clbCt++;
  }

  //compute position
  computePosInCounts(midKey);
  computePosInCounts(leftKey);
  computePosInCounts(rightKey);
  
  computePosInMeters(midKey, 0.15);
  computePosInMeters(rightKey, 0.15);
  computePosInMeters(leftKey, 0.15);

  #ifdef PIANO

  double kWall = 300;
  double bottomThresh = 0.02; //Changing this will require you to change it in Processing too.
 
  pianoRender(midKey, 30, 0.2, bottomThresh, kWall, 60*(0 - bottomThresh));
  pianoRender(leftKey, 50, 0.3, bottomThresh, kWall, 50*(0 - bottomThresh));
  pianoRender(rightKey, 30, 0.1, bottomThresh, kWall, 40*(0 - bottomThresh));

  #endif


  #ifdef HARPSICHORD

  double kWall = 300;
  double bottomThresh = 0.02; //Changing this will require you to change it in Processing too.
 
  harpsichordRender(midKey, 30, 0.2, bottomThresh, kWall, 50*(0 - bottomThresh));
  harpsichordRender(leftKey, 50, 0.3, bottomThresh, kWall, 50*(0 - bottomThresh));
  harpsichordRender(rightKey, 30, 0.1, bottomThresh, kWall, 35*(0 - bottomThresh));

  #endif

  computeForce(midKey.force, midKey.pwmPin, midKey.dirPin);
  computeForce(leftKey.force, leftKey.pwmPin, leftKey.dirPin);
  computeForce(rightKey.force, rightKey.pwmPin, rightKey.dirPin);

  String leftxh = String(leftKey.xh);
  String midxh = String(midKey.xh);
  String rightxh = String(rightKey.xh);
  String leftvel = String(leftKey.dxh_filt);
  String midvel = String(midKey.dxh_filt);
  String rightvel = String(rightKey.dxh_filt);
  String printStatement = leftxh + "," + midxh + "," + rightxh 
      + "," + leftvel + "," + midvel + "," + rightvel;

if (count <= 0) {
    Serial.println(printStatement);
    count = 20;
  }
count--; 
  
  
}

/*
 * This function computes PWM output for a given force, 
 * output pin, and direction pin
 */
void computeForce(double force, int outpin, int dir) {
  // Define kinematic parameters
   double rh = 0.15;
   double rp = 0.005;   //[m]
   double rs = 0.075;   //[m] 
   double Tp = force*rh*rp/rs;
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dir, HIGH);
  } else {
    digitalWrite(dir, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  double duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  unsigned int output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(outpin,output);  // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
