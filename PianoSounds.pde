import processing.serial.*;
import processing.sound.*;


String keyboard = "Piano";
//String keyboard = "Harpsichord";


Serial myPort;        // The serial port
int serialNumber = 1;

SoundFile[] file = new SoundFile[3];
boolean[] playing = new boolean[3];

float bottomThresh; // Changing this will require you to change it in main.cpp too.
float extraThresh;
float stopThresh;
float maxVel = 1;  //m/s

void setup () {
  
  //setup global variables
  if (keyboard.equals("Piano")) {
    bottomThresh = 0.02;
    extraThresh = 0.003;
    stopThresh = 0.015;
    file[0] = new SoundFile(this, "C.wav");
    file[1] = new SoundFile(this, "D.wav");
    file[2] = new SoundFile(this, "E.wav");
  } else if (keyboard.equals("Harpsichord")) {
    bottomThresh = 0.02*3/4;
    extraThresh = 0;
    stopThresh = 0.02*2/3;
    file[0] = new SoundFile(this, "C_Harpsichord.wav");
    file[1] = new SoundFile(this, "D_Harpsichord.wav");
    file[2] = new SoundFile(this, "E_Harpsichord.wav");
  }
  
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[serialNumber], 9600);  //

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
  
  playing[0] = false;
  playing[1] = false;
  playing[2] = false;
}

void draw () {
  background(0);

// check if a key is pressed - if so change color to Cardinal Red  
  if(playing[0]){
    stroke(140,21,21);
    fill(140,21,21);
    rect(170,75,80,250);
  }

// if key is not pressed - remain white in color
  else{
    if (keyboard.equals("Piano")) {
      stroke(255);
      fill(255);
    } else if (keyboard.equals("Harpsichord")) {
      stroke(71,46,33);
      fill(71,46,33);
    }
  rect(170,75,80,250);    
  }
  
  if(playing[1]){
    stroke(140,21,21);
    fill(140,21,21);
    rect(260,75,80,250);
  }
  
  else{
    if (keyboard.equals("Piano")) {
      stroke(255);
      fill(255);
    } else if (keyboard.equals("Harpsichord")) {
      stroke(71,46,33);
      fill(71,46,33);
    }    
  rect(260,75,80,250);
  }
  
  if(playing[2]){
    stroke(140,21,21);
    fill(140,21,21);
    rect(350,75,80,250);
  }
  
  else{
    if (keyboard.equals("Piano")) {
      stroke(255);
      fill(255);
    } else if (keyboard.equals("Harpsichord")) {
      stroke(71,46,33);
      fill(71,46,33);
    }
  rect(350,75,80,250);
  }  

// create black keys between C-D and D-E keys  
  if (keyboard.equals("Piano")) {
    stroke(0);
    fill(0);
  } else if (keyboard.equals("Harpsichord")) {
    stroke(163,133,112);
    fill(163,133,112);
  }
  rect(230,75,50,135);
  rect(320,75,50,135);
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  //println(inString);
  if (inString != null) {
    String[] s = splitTokens(inString, ",");

    for (int i = 0; i < 3; i++) {
      if (float(s[i]) > bottomThresh - extraThresh) {
        if (!playing[i]) {
          if (keyboard.equals("Piano")) {
            float volume = map(float(s[i+3]), 0.0, maxVel, 0.0, 1.0);
            if (volume > 1.0) volume = 1.0;
            file[i].amp(volume);
          }
          file[i].play();
          playing[i] = true;
        }
      } else if (float(s[i]) < stopThresh) {
        if (playing[i]) {
          file[i].stop();
          playing[i] = false;
        }
      }
    }
  }
}
