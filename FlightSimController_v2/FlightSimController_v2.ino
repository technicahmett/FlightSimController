// Requires Arduino Joystick Library https://github.com/MHeironimus/ArduinoJoystickLibrary

// xx Buton ve xx Joystick uygulaması: 25.12.2020 AhmetT
// Arduino Leonardo 

//#include <Keyboard.h> //Klavye için ekstra tuslar denenecek
#include <Mouse.h> //Mouse un scroll özelligi denenecek

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <Joystick.h>
//#include <Keypad.h>
Joystick_ Joystick;

/*deneme olmadı
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'0','1','2','3'},
  {'4','5','6','7'},
  {'8','9','A','B'},
  {'C','D','E','F'}
};
byte rowPins[ROWS] = {3, 2, 1, 0}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {7, 6, 5, 4}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
olmadı*/


//Joystickler
int JoystickX; //roll
int JoystickY; //pitch
int JoystickZ; //yaw
int Throttle; //thr
int Slider1; //look left/right
int Slider2; //look up/down
int Slider3; //landing gear up/down
int Slider4; //pitch trim (elevator trim)
int Aux1; //yedek

//Butonlar
int currentButtonState0;
int lastButtonState0;
int currentButtonState1;
int lastButtonState1;
int currentButtonState2;
int lastButtonState2;
int currentButtonState3;
int lastButtonState3;
int currentButtonState4;
int lastButtonState4;
int currentButtonState5;
int lastButtonState5;
int currentButtonState6;
int lastButtonState6;
int currentButtonState7;
int lastButtonState7;


// Rotary Encoder ile mouse scroll zoom icin
Encoder myEnc(2, 3);

// Variables will change:
long previousMillis = 0;        // will store last time LED was updated

// Distance to scroll
long scrollDistance = 0;
int sensitivity = 1;

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 100;           // interval at which to blink (milliseconds)


void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
 // pinMode(A7, INPUT_PULLUP);
  pinMode(A8, INPUT_PULLUP); 
  pinMode(A9, INPUT_PULLUP); 
  pinMode(A10, INPUT_PULLUP); 
  pinMode(A11, INPUT_PULLUP);
  Serial.begin(9600);  

//encoder-scroll
long position  = -999;

// Initialize Joystick Library
  Joystick.begin();
  Joystick.setXAxisRange(0, 850); //hall sensor icin
  Joystick.setYAxisRange(560, 850); //hall sensor icin
  Joystick.setZAxisRange(0, 1024);
  Joystick.setThrottleRange(0, 1023);
  Joystick.setRxAxisRange(0, 1024); 
  Joystick.setRyAxisRange(0, 1024);
  Joystick.setRzAxisRange(0, 1024);
}

void loop() {
  
// Read Joystick
  JoystickX = analogRead(A1); // Hall effect sensor connects to this analog pin
  JoystickX = map(JoystickX,573,892,0,1023); // 0-255 ve 0-512 denenecek
  
  JoystickY = analogRead(A2); // Hall effect sensor connects to this analog pin

// Read Rudder Pedals
  JoystickZ = analogRead(A3); // Hall effect sensor connects to this analog pin

// Read Throttle
  Throttle = analogRead(A0); // Potentiometer signal connects to this analog pin
 // Throttle = map(Throttle,15,1023,0,255); // 0-255 ve 0-512 denenecek
 
// Read Sliders
  Slider1 = analogRead(A4); // Potentiometer signal or Hall effect sensor connects to this analog pin
  Slider2 = analogRead(A5); // Potentiometer signal or Hall effect sensor connects to this analog pin
  Slider3 = analogRead(A6); // Potentiometer signal or Hall effect sensor connects to this analog pin
 // Slider4 = analogRead(A7); // Potentiometer signal or Hall effect sensor connects to this analog pin
  Aux1 = analogRead(A8); // Potentiometer signal or Hall effect sensor connects to this analog pin

//encoder-scroll
  unsigned long currentMillis = millis();
  long newPos = myEnc.read();
  
  if(currentMillis - previousMillis > interval) {
    // Last interval time
    previousMillis = currentMillis;   
    
    scrollDistance = newPos;
    // Send scroll distance to computer
    scrollDistance = scrollDistance * sensitivity;
    // Scroll mouse if connected
   // if(digitalRead(4) == LOW){// eger scrollu bir butonla aç kapa yapmak istersek
      Mouse.begin();
      Mouse.move(0,0, scrollDistance);
    }
    //else {
      Mouse.end();
    //}    
    // Reset scroll position
    myEnc.write(0);
    //encoder bitti

/*Serial.print("__A0: ");
Serial.print(analogRead(A0));
Serial.print("___throttle:  ");
Serial.print(Throttle);
Serial.print("__A1: ");
Serial.print(analogRead(A1));
Serial.print("____JoystickX:  ");
Serial.println(JoystickX);*/

/*
Serial.println("A2");
Serial.println(analogRead(A2));
Serial.println("A3");
Serial.println(analogRead(A3));*/

// Read Switches
int currentButtonState0 = !digitalRead(7); // Button 1
  if (currentButtonState0 != lastButtonState0)
  {
  Joystick.setButton(0, currentButtonState0);
  lastButtonState0 = currentButtonState0;
  }

int currentButtonState1 = !digitalRead(5); // Slider4 no button 2
  if (currentButtonState1 != lastButtonState1)
  {
  Joystick.setButton(1, currentButtonState1);
  lastButtonState1 = currentButtonState1;
  }
  
int currentButtonState2 = !digitalRead(6); // Button 3
  if (currentButtonState2 != lastButtonState2)
  {
  Joystick.setButton(2, currentButtonState2);
  lastButtonState2 = currentButtonState2;
  }

int currentButtonState3 = !digitalRead(A8); // Aux1 no button 4
  if (currentButtonState3 != lastButtonState3)
  {
  Joystick.setButton(3, currentButtonState3);
  lastButtonState3 = currentButtonState3;
  }
  
int currentButtonState4 = !digitalRead(A9); // Button 5
  if (currentButtonState4 != lastButtonState4)
  {
  Joystick.setButton(4, currentButtonState4);
  lastButtonState4 = currentButtonState4;
  } 

int currentButtonState5 = !digitalRead(A10); // Button 6
  if (currentButtonState5 != lastButtonState5)
  {
  Joystick.setButton(5, currentButtonState5);
  lastButtonState5 = currentButtonState5;
  } 
/*  
int currentButtonState6 = !digitalRead(A11); // Button 7
  if (currentButtonState6 != lastButtonState6)
  {
  Joystick.setButton(6, currentButtonState6);
  lastButtonState6 = currentButtonState6;
  }  */

// Output Controls
  Joystick.setXAxis(JoystickX);
  Joystick.setYAxis(JoystickY); 
  Joystick.setZAxis(JoystickZ);
  Joystick.setThrottle(Throttle);
  Joystick.setRxAxis(Slider1);
  Joystick.setRyAxis(Slider2);
  Joystick.setRzAxis(Slider3);
//digerleri icin belirlenecek

  Joystick.sendState();

} 
