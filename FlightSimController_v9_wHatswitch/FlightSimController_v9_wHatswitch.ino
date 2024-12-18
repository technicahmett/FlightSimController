// Thanks for Arduino Joystick Library. https://github.com/MHeironimus/ArduinoJoystickLibrary

// 32 Buton ve 7 Joystick uygulaması: 02.022.2021 AhmetT
// Arduino Leonardo ve Micro
//10.04.2021

////////////Micro Pinout//////////
/* D2:SDA
 * D3:SCL
 * D14:ENCODER
 * D16:ENCODER
 * D8:A8=ANALOG 6
 * D9:A9=ANALOG 7
 * D10:A10=ANALOG 5
 * A0=ANALOG 4
 * A1=ANALOG 3
 * A2=ANALOG 2
 * A3=ANALOG 1
 * D4=Hat Switch #0 UP
 * D5=Hat Switch #0 RIGHT
 * D6=Hat Switch #0 DOWN
 * D7=Hat Switch #0 LEFT
 * D15=BUTON 
 */

//#include <Keyboard.h> //Gerek yok
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <Mouse.h> //Mouse un scroll özelligi
#include <Joystick.h>
//#include <Keypad.h>
#include <Keypad_I2C.h>

Joystick_ Joystick;

////////////////////////PCF8574 i2c adresler
#define I2CADDR 0x20 //birinci i2c butonların adresi
#define I2CADDR2 0x21 //ikinci i2c butonların adresi

////////////////////Joystickler/////////////////
int JoystickX; //roll
int JoystickY; //pitch
int JoystickZ; // yaw 
int Throttle; //Throttle
int RotateX; //pitch trim (elevator trim)
int RotateY; //roll trim
int RotateZ; //landing gear up/down


/* Bu butonlar yerine 2 adet PCF8574 ile 2x16buton eklendi.
//////////////Butonlar//////////////////
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
int currentButtonState8;
int lastButtonState8;
int currentButtonState9;
int lastButtonState9;
int currentButtonState10;
int lastButtonState10;
int currentButtonState11;
int lastButtonState11;
int currentButtonState12;
int lastButtonState12;
int currentButtonState13;
int lastButtonState13;
int currentButtonState14;
int lastButtonState14;
int currentButtonState15;
int lastButtonState15;
int currentButtonState16;
int lastButtonState16;
int currentButtonState17;
int lastButtonState17;
int currentButtonState18;
int lastButtonState18;
int currentButtonState19;
int lastButtonState19;
int currentButtonState20;
int lastButtonState20;
int currentButtonState21;
int lastButtonState21;
*/
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

/////////////////////////////////PCF8574 2x16 buton////////////////////////
const int joySwitch = 11; // 11 numaralı IO portu

int lastButtonState = 0;

const byte ROWS = 4; //dört satır 
const byte COLS = 4; //dört sütun
//define the cymbols on the buttons of the keypads

char buttons[ROWS][COLS] = { /////////Birinci grup matris
  {0, 1, 2, 3},
  {4, 5, 6, 7},
  {8, 9, 10, 11},
  {12, 13, 14, 15},
};
char buttons2[ROWS][COLS] = { /////////İkinci grup matris
  {16, 17, 18, 19},
  {20, 21, 22, 23},
  {24, 25, 26, 27},
  {28, 29, 30, 31},
};

////////////////////Birinci PCF8574'e ait matris tuş sıralaması////////////////
byte rowPins[ROWS] = {0, 1, 2, 3}; ///////satırlar
byte colPins[COLS] = {4, 5, 6, 7}; ///////sütunlar
////////////////////Birinci PCF8574'e ait matris tuş sıralaması////////////////
byte rowPins2[ROWS] = {0, 1, 2, 3}; ///////satırlar
byte colPins2[COLS] = {4, 5, 6, 7}; ///////sütunlar


////////////////////Birinci PCF8574 ün yapısı yüklenir///////////////////
Keypad_I2C yokeButtons = Keypad_I2C(makeKeymap(buttons), rowPins, colPins, ROWS, COLS, I2CADDR); 
////////////////////İkinci PCF8574 ün yapısı yüklenir/////////////////////
Keypad_I2C yokeButtons2 = Keypad_I2C(makeKeymap(buttons2), rowPins2, colPins2, ROWS, COLS, I2CADDR2); 
////////////////////////////////////////////////////////////////////////////////////


////////////////////// Rotary Encoder ile mouse scroll zoom icin///////////////
Encoder myEnc(16,14);/////denenecek

// Variables will change:
long previousMillis = 0;        // will store last time LED was updated

////// Distance to scroll
long scrollDistance = 1;
int sensitivity = 1;

////// the follow variables is a long because the time, measured in miliseconds,
///// will quickly become a bigger number than can be stored in an int.
long interval = 10;           // interval at which to blink (milliseconds)
//////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  //pinMode(11, INPUT_PULLUP); //Microda yok.
  //pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  //pinMode(16, INPUT_PULLUP);
  //pinMode(A0, INPUT_PULLUP);
  //pinMode(A1, INPUT_PULLUP);
  //pinMode(A2, INPUT_PULLUP);
  //pinMode(A3, INPUT_PULLUP);
//  pinMode(A4, INPUT_PULLUP);  //Microda yok.
//  pinMode(A5, INPUT_PULLUP);  //Microda yok.
//  pinMode(A6, INPUT_PULLUP); //Microda 4
//  pinMode(A7, INPUT_PULLUP); //Microda 6
  //pinMode(A8, INPUT_PULLUP); 
  //pinMode(A9, INPUT_PULLUP); 
  //pinMode(A10, INPUT_PULLUP); 
// pinMode(A11, INPUT_PULLUP); //Microda yok.


  Serial.begin(9600); //Seri başlar  
  Joystick.begin();   //joystick başlar
  Wire.begin( );      //i2c başlar
///////////////////////////////////////////

/////////////encoder-scroll///////////////
long position  = -999;//-999
//////////////////////////////////////////


/////////////////Initialize Joystick Library////////////////////
  Joystick.setXAxisRange(0, 850); //hall sensor icin
  Joystick.setYAxisRange(560, 850); //hall sensor icin
 // Joystick.setRudderRange(0, 1024);
//  Joystick.setZAxisRange(0, 1024);
  Joystick.setThrottleRange(0, 1023);
  Joystick.setRxAxisRange(0, 1024); 
  Joystick.setRyAxisRange(0, 1024);
  Joystick.setRzAxisRange(0, 1024);
  Joystick.setAcceleratorRange(0, 1024);

  pinMode(joySwitch, INPUT_PULLUP);
  yokeButtons.begin(); //birinci i2c butonları başlatır
  yokeButtons2.begin(); //ikinci i2c butonları başlatır
}


void loop() {
////////////////////////encoder çalışır///////////////////////
  unsigned long currentMillis = millis();
  long newPos = myEnc.read();
    
  if(currentMillis - previousMillis > interval) {
    // Last interval time
    previousMillis = currentMillis;   
    scrollDistance = newPos;
    // Send scroll distance to computer
    scrollDistance = scrollDistance * sensitivity;
    // Scroll mouse if connected
    
//  if(digitalRead(4) == LOW){ // D4 okun ur ve eğer Low ise Encoder başlar.
      Mouse.begin();
      Mouse.move(0,0, scrollDistance);
  // }
    //else { Mouse.end();}     
    
    //////// Reset scroll position
  myEnc.write(0);   
  }
  Serial.println(scrollDistance);
  /////////////////////encoder biter/////////////////////////////////
  
   checkButtons();  
//////////////////////////Read Joystick/////////////////////////////
  JoystickX = analogRead(A2); // Hall effect sensor connects to this analog pin
 // JoystickX = map(JoystickX,573,892,0,1023); // 0-255 ve 0-512 denenecek
  JoystickY = analogRead(A1); // Hall effect sensor connects to this analog pin
 // JoystickY = map(JoystickY,573,892,0,1023); // 0-255 ve 0-512 denenecek
  JoystickZ = analogRead(A0); // Potentiometer signal or Hall effect sensor connects to this analog pin
//////////////////////////Read Throttle////////////////////////////
  Throttle = analogRead(A3); // Potentiometer signal connects to this analog pin
 // Throttle = map(Throttle,15,1023,0,255); // 0-255 ve 0-512 denenecek
///////////////////////////Read Sliders/////////////////////////////
  RotateX = analogRead(A10); // Potentiometer signal or Hall effect sensor connects to this analog pin
  RotateY = analogRead(A8); // Potentiometer signal or Hall effect sensor connects to this analog pin
  RotateZ = analogRead(A9); // Potentiometer signal or Hall effect sensor connects to this analog pin

/*
  ////////////////////////////Read Switches/////////////////////////  
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
  */
/*  
int currentButtonState6 = !digitalRead(A11); // Button 7
  if (currentButtonState6 != lastButtonState6)
  {
  Joystick.setButton(6, currentButtonState6);
  lastButtonState6 = currentButtonState6;
  }  */
///////////////////////////HatSwich çalışır/////////////////////////////
//int hatSwitch = 0; // Birinci HatSwich. Eğer ikinci istenirse "1" yapılır
 
  if ((lastButtonState0 == 0)
      && (lastButtonState1 == 0)
      && (lastButtonState2 == 0)
      && (lastButtonState3 == 0)
      && (lastButtonState4 == 0)) 
      {
      Joystick.setHatSwitch(0, -1); ////// HatSwich e basılmadığı zaman HatSwich sıfırlanır. 
      Joystick.setHatSwitch(1, -1); 
      }
//Joystick.setHatSwitch(hatSwitch, -1)::::(x, y) x:HatSwich kodu. yukardan da değişebilir yada buraya 0, 1 yazılarak istenen HatSwich seçilir.
//                                               y: HatSwitch bakış açısıdır. -1 pasif. 0 0derece yani ileri, 90 90derece......

int currentButtonState0 = !digitalRead(4); 
  if (currentButtonState0 != lastButtonState0)
  {Joystick.setHatSwitch(0, 0);   //Birinci Hatswich, 0 derece
   } 
   int currentButtonState1 = !digitalRead(5); 
  if (currentButtonState1 != lastButtonState1)
  {Joystick.setHatSwitch(0, 90);  //Birinci Hatswich, 90 derece
   } 
int currentButtonState2 = !digitalRead(6); 
  if (currentButtonState2 != lastButtonState2)
  {Joystick.setHatSwitch(0, 180); //Birinci Hatswich,  180 derece
   } 
int currentButtonState3 = !digitalRead(7); 
  if (currentButtonState3 != lastButtonState3)
  {Joystick.setHatSwitch(0, 270);  //Birinci Hatswich, 270 derece
   } 

   int currentButtonState4 = !digitalRead(15); 
  if (currentButtonState4 != lastButtonState4)
  {Joystick.setHatSwitch(1, 270);  //Birinci Hatswich, 270 derece
   } 
/////////////////////////////HatSwich Sonu////////////////////////////////////////
  
/////////////////////Output Controls/////////////////////////
  Joystick.setXAxis(JoystickX); //Elevator
  Joystick.setYAxis(JoystickY); //Aileron
  Joystick.setZAxis(JoystickZ); //Rudder
  Joystick.setThrottle(Throttle); //Throttle
  Joystick.setRxAxis(RotateX); //Pitch Trim
  Joystick.setRyAxis(RotateY); //Roll Trim
  Joystick.setRzAxis(RotateZ); //Landing Gear Up/Down
  Joystick.sendState();

} 
void checkButtons(void) {  
  ///////////////////////   butonlar okunur   ///////////////////////////
  int currentButtonState = !digitalRead(joySwitch); 
  if (currentButtonState != lastButtonState){
    Joystick.setButton(12, currentButtonState);
    lastButtonState = currentButtonState;
  }

  ////////////////////   birinci grup 16 lı butonlar okunur   ////////////////////
  if (yokeButtons.getKeys())
  {
    for (int i=0; i<LIST_MAX; i++)   
        {
           if (yokeButtons.key[i].stateChanged )   
            {
            switch (yokeButtons.key[i].kstate) {  
                    case PRESSED:
                    case HOLD:
                              Joystick.setButton(yokeButtons.key[i].kchar, 1);
                              break;
                    case RELEASED:
                    case IDLE:
                              Joystick.setButton(yokeButtons.key[i].kchar, 0);
                              break;
            }
            }
        }
  }
////////////////////  ikinci grup 16 lı butonlar okunur   ////////////////////
 else if (yokeButtons2.getKeys())
 {
   for (int k=0; k<LIST_MAX; k++)   
        {
           if (yokeButtons2.key[k].stateChanged )   
            {
            switch (yokeButtons2.key[k].kstate) {  
                    case PRESSED:
                    case HOLD:
                              Joystick.setButton(yokeButtons2.key[k].kchar, 1);
                              break;
                    case RELEASED:
                    case IDLE:
                              Joystick.setButton(yokeButtons2.key[k].kchar, 0);
                              break;
            }
            }
        }
       }
}
