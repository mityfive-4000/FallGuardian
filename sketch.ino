// Sketch for the Fall Guardian
// Made for Intelhacks Devpost Challenge 2017

#include <TinyGPS++.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <CurieBLE.h>
#include <CurieIMU.h>
#include "rgb_lcd.h"

#define HR_NAME "M2" // Bingo M2
#define CTRL_CHRC_UUID "00002a5f-0000-1000-8000-00805f9b34fb"
#define HR_CHRC_UUID "6e400009-b5a3-f393-e0a9-e50e24dcca9e"
#define STEP_CHRC_UUID "6e400005-b5a3-f393-e0a9-e50e24dcca9e"

#define MOBILE_NUMBER "+440000000000"

// Pins
const int ledPin = 2;
const int soundPin = 3;
const int btnAPin = 4;
const int btnBPin = 5;
const int gnssRx = 7;
const int gnssTx = 8;
const int gsmPwrPin = 9;

// Constants
const int CTRL_VIBRATE = 0x02;
const int CTRL_NOTIFY = 0x04;
const int CTRL_HR_OPENSERVICE = 0x22;
const int CTRL_HR_START = 0x31;
const int UTC_OFFSET = 2;
const int GSMTIMEOUT = 8000;
const int SERBUFF = 64;
const int SMSBUFF = 256;
const int HEARTRATEMIN = 45;
const int HEARTRATEMAX = 160;
const int MEASUREDELAY = 15000;
const int CHECKTIMEOUT = 60000;
const int MENUTIMEOUT = 6000;
const int LCDTIMEOUT = 8000;
const int BLETIMEOUT = 4000;
const int FALLTIMEOUT = 1000;
const int SAFETYTIMEOUT = 10000;
const int MENUITEMS = 6;
const int MESSAGES = 7;
const int GEOFENCE_OUTER = 500;
const int GEOFENCE_INNER = 50;
const int REPORT = 100;
const char* TEXTMESSAGE[]={"Please call me","Come over now","I'm doing fine","I'm not well","I'm at home","Back very soon","I'm back later"};
const double HOME_LAT = 48.85826;
const double HOME_LNG = 2.294516;

// Global objects
SoftwareSerial gnssSerial(gnssRx, gnssTx);
unsigned long seconds;
unsigned long fallTimeout;
bool falling,fallEvent;
bool atHome;
bool sentHrWarning;
unsigned int heartRate;
unsigned int minutes;
unsigned long hrTimeout, measureTimeout;
unsigned long smsTimeout;
unsigned long menuTimeout, lcdTimeout;
bool lcdOn;
bool hrFound;
bool smsQuery;
unsigned int menuPos;
unsigned char smsBuffer[SMSBUFF];
unsigned char savedText[SMSBUFF];
unsigned int gnssDistance;

rgb_lcd lcd;
BLEDevice hrMonitor;
BLECharacteristic ctrlCharacteristic,hrCharacteristic,stepCharacteristic;
TinyGPSPlus gnss;  

// Setup ------------------------------------------------------

void setup() 
{
  byte arrowLeft[8] = {
    0b00000,
    0b00100,
    0b01000,
    0b11111,
    0b01000,
    0b00100,
    0b00000,
    0b00000
  };

  byte arrowRight[8] = {
    0b00000,
    0b00100,
    0b00010,
    0b11111,
    0b00010,
    0b00100,
    0b00000,
    0b00000
  };

  byte rect[8] = {
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b00000,
    0b00000
  };

  // Initialise input pins
  pinMode(btnAPin, INPUT_PULLUP); // physical button, normally high
  pinMode(btnBPin, INPUT); // touch button, normally low    
  
  // Initialise output pins
  pinMode(ledPin, OUTPUT);
  pinMode(soundPin, OUTPUT);
  pinMode(gsmPwrPin, OUTPUT);
  digitalWrite(gsmPwrPin,LOW);
  
  // Initialise LCD
  lcd.begin(16,2);    
  lcd.setRGB(0,128,255);    
  lcd.print("Hello!");
  lcd.setCursor(0,1);
  lcd.print("Starting ");
  lcd.createChar(1,arrowLeft);
  lcd.createChar(2,rect);
  lcd.createChar(3,arrowRight);
  lcd.createChar(4,rect);
  
  // Beep and LED flash
  digitalWrite(ledPin,HIGH);
  beep(10);
  digitalWrite(ledPin,LOW);

  // Initialise variables
  fallEvent=falling=false;;
  fallTimeout=0;
  menuPos=menuTimeout=0;
  hrTimeout=measureTimeout;
  smsTimeout=CHECKTIMEOUT/2;
  hrFound=smsQuery=false;
  savedText[0]=0;
  heartRate=60;
  minutes=99;
  gnssDistance=0;
  atHome=true;
  sentHrWarning=false;
  
  // Initialise Serial Ports
  lcd.setCursor(9,1);
  lcd.print("Serial...");
  Serial.begin(19200);
  //while(!Serial);
  Serial1.begin(19200);
  gnssSerial.begin(9600);

  // Avoid stuck touch button
  lcd.setCursor(9,1);
  lcd.print("Button...");
  while(digitalRead(btnBPin)==HIGH);
  
  // Start GSM
  lcd.setCursor(9,1);
  lcd.print("GSM...   ");
  if(!gsmPowerOn()) {
    lcd.setCursor(0,0);
    lcd.print("GSM not working!");
    promptRestart();
  }
  if(!gsmInitialise()) {
    lcd.setCursor(0,0);
    lcd.print("GSM bad response");
    promptRestart();
  }
  
  // Initialise BLE
  lcd.setCursor(9,1);
  lcd.print("BLE...");
  lcd.setCursor(0,1);
  
  if(!(hrFound=findHrMonitor())) {
    lcd.setRGB(255,0,0);
    lcd.print("No HR band found");
    //beep(100);
    delay(2000);
    lcd.setRGB(0,128,255);  
  }
  
  // Initialise the IMU
  lcd.setCursor(0,1);
  lcd.print("Starting IMU... ");
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(4);
  CurieIMU.attachInterrupt(imuCallback);

  // Enable Freefall Detection
  CurieIMU.setDetectionThreshold(CURIE_IMU_FREEFALL,500);
  CurieIMU.setDetectionDuration(CURIE_IMU_FREEFALL,50); //ms
  CurieIMU.interrupts(CURIE_IMU_FREEFALL);

  // Enable Shock Detection
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK,500); //mg
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK,20); //ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK); 
    
  // Done!
  lcd.clear();
  showMenu();
  
}


// Callbacks --------------------------------------------------

static void bleNotify(BLEDevice bleDev, BLECharacteristic bleCharact) {
  int i,n;
  Serial.println("BLE notification");
  Serial.print(bleCharact.uuid());
  Serial.print("-");
  Serial.print(n=bleCharact.valueLength());
  if(n>0) {
    Serial.print("=");
    for(i=0;i<n;i++)
      Serial.print(bleCharact.value()[i],HEX);
  }
  Serial.println("");

  //heartRate=bleCharact.value()[0];
  heartRate=80;
}

static void imuCallback(void)
{
  if(CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
    if(falling) {
      lcd.setRGB(255,0,0);
      fallEvent=true;    
    }
  }
  if(CurieIMU.getInterruptStatus(CURIE_IMU_FREEFALL)) {
    if(!fallEvent) lcd.setRGB(255,128,0);
    falling=true;
    lcdTimeout=fallTimeout=millis();
  }  
}


// Main loop --------------------------------------------------

void loop() 
{
    int n;
    
    // Serial passthrough to allow control of GSM via serial terminal
    while(Serial.available()||Serial1.available()) {
        if(Serial1.available()) Serial.write(Serial1.read());
        if(Serial.available()) Serial1.write(Serial.read());
    }

    // Read GNSS data and pass to TinyGPS++
    while(gnssSerial.available())
      gnss.encode(gnssSerial.read());
    
    // Check backlight timeout and fade out
    if((lcdOn)&&((n=(int)((millis()-lcdTimeout))-LCDTIMEOUT)>0)) {
      lcd.setRGB(0,255-n/4,0);
      if(n>1000) {
        lcd.setRGB(0,0,0);
        lcdOn=false;
      }
    }

    // Check menu timeout
    if((menuPos>0)&&(millis()-menuTimeout)>MENUTIMEOUT) {
      menuPos=0;
      showMenu();
    }

    // Check menu cycling
    if(digitalRead(btnBPin)==HIGH) {
      digitalWrite(ledPin,HIGH);
      menuPos=(menuPos+1)%MENUITEMS;
      showMenu();
      beep(10);
      delay(200);
      digitalWrite(ledPin,LOW); 
    }

    // Check for panic press or menu action
    if(digitalRead(btnAPin)==LOW) {
      digitalWrite(ledPin,HIGH);
      if(menuPos!=0) beep(10);
      performAction(menuPos);
      menuPos=0;
      showMenu();
      while((digitalRead(btnAPin)==LOW)||(digitalRead(btnBPin)==HIGH));
      digitalWrite(ledPin,LOW);
    }

    if(menuPos==0) { // Only do these if we're not navigating the menu
      
      // Send a status report if requested
      if(smsQuery) {
          gsmSendSMS(REPORT);  
          smsQuery=false;
        }
          
      // Check for new SMS
      if((millis()-smsTimeout)>CHECKTIMEOUT) {
        digitalWrite(ledPin,HIGH);
        Serial.println("Checking for new SMS");
        smsTimeout=millis();
        if(gsmMessages()>0)
          if(readMessages()) { // New message received
            if(hrFound) buzzHrMonitor();
            lcd.setRGB(0,128,255);
            lcd.clear();
            lcd.setCursor(0,1);
            showMessage();
            showMenu();
          }
            
        digitalWrite(ledPin,LOW);      
      }
  
      // Start heart rate measurement
      if((millis()-hrTimeout)>CHECKTIMEOUT) {
        digitalWrite(ledPin,HIGH);
        Serial.println("Reading heart rate");
        if(getHeartRate())
          measureTimeout=millis();
        hrTimeout=millis();
        digitalWrite(ledPin,LOW);      
      }
  
      // Check heart rate measurement results (updated by callback)
      if(((millis()-measureTimeout)>MEASUREDELAY)&&(measureTimeout>0)) {
        if(!sentHrWarning&&((heartRate<HEARTRATEMIN)||(heartRate>HEARTRATEMAX))) {
          Serial.println("Heart rate exceeds limit");
          if(!userCancel()) {
            gsmSendSMS(REPORT+2);
            sentHrWarning=true;
          }
          showMenu();
        }
        measureTimeout=0;
      }

      // Perform GNSS checks, once per second, if we have data
      if(((millis()/1000)!=seconds)&&(gnss.time.isUpdated())) {
        
        // Display GNSS time, if the minute changed
        if((n=gnss.time.minute())!=minutes) {
          minutes=n;
          lcd.setCursor(11,0);
          n=((gnss.time.hour())+UTC_OFFSET)%24;
          if(n<10) lcd.print("0");
          lcd.print(n);
          lcd.print(":");
          if(minutes<10) lcd.print("0");
          lcd.print(minutes);
        }
  
        // Check distance from home location
        if((gnss.location.lat()!=0)&&(gnss.location.lng()!=0)) {
          gnssDistance=(unsigned int)(gnss.distanceBetween(gnss.location.lat(),gnss.location.lng(),HOME_LAT,HOME_LNG));
          
          if(atHome&&(gnssDistance>GEOFENCE_OUTER)) {
            atHome=false;
            Serial.println("Moved outside geofence, distance ");
            Serial.print(gnssDistance);
            Serial.println("m");
            gsmSendSMS(REPORT+3);
          }
    
          if(!atHome&&(gnssDistance<GEOFENCE_INNER)) {
            atHome=true;
            Serial.println("Returned inside geofence");
          }
        }
               
        seconds=millis()/1000;      
      } 
      
    }
        
    // Cancel a fall event if it didn't end in shock
    if((fallTimeout!=0)&&((millis()-fallTimeout))>FALLTIMEOUT) {
      falling=false;
      fallTimeout=0;
      lcd.setRGB(0,255,0);
      lcdTimeout=millis();
      lcdOn=true;
    }

    // Check for complete fall+shock event
    if(fallEvent) {
      Serial.println("Fall event");
      lcd.setRGB(255,0,0);
      if(!userCancel()) {
        gsmSendSMS(REPORT+4);
        alarm();
      }
      fallEvent=falling=false;
      fallTimeout=0;
      menuPos=0;
      showMenu();
      while(digitalRead(btnAPin)==LOW);
      delay(200);
    }

    //delay(100);
}


// General routines -------------------------------------------

void showMenu(void) {
  //lcd.setRGB(0,n==0?64:255,0);
  lcd.setRGB(0,255,0);
  lcd.setCursor(0,0);
  switch(menuPos) {
    case 0: lcd.print("Running.        "); break;
    case 1: lcd.print("Read message    "); break;
    case 2: lcd.print("Send message    "); break;
    case 3: lcd.print("Find wristband  "); break;
    case 4: lcd.print("Check heart rate"); break;
    case 5: lcd.print("Show GNSS info  "); break;
  }
  lcd.setCursor(0,1);
  lcd.write(1);
  switch(menuPos) {
    case 0: lcd.print("PANIC     MENU"); break;
    default: lcd.print("SELECT    NEXT"); break;
  }
  lcd.write(3);
  minutes=99;
  menuTimeout=lcdTimeout=millis();
  lcdOn=true;    
}

void performAction(int n) {
  lcd.clear();
  lcd.setCursor(0,1);
  switch(n) {
    case 0:
      if(panicPress()) {
        Serial.println("Panic button activated");
        lcd.setRGB(255,0,0);
        lcd.setCursor(0,0);
        lcd.print("Sending message!");
        digitalWrite(soundPin,HIGH);
        digitalWrite(ledPin,HIGH);  
        gsmSendSMS(REPORT+1);
        digitalWrite(ledPin,LOW);
        alarm();               
      }
      else showMenu();
      break;
    case 1: showMessage(); break;
    case 2: sendMessage(); break;
    case 3: findOrBuzzHr(); break;
    case 4: findHr(); break;
    case 5: showGnssInfo(); break;   
  }
}

bool userCancel(void) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.write(1);
  lcd.print("HOLD TO CANCEL!");
  beep(100);  
  return !progressBar(400,LOW);
}

bool panicPress(void) {
  lcd.setRGB(255,128,0);
  lcd.setCursor(0,0);
  lcd.write(1);
  lcd.print("HOLD FOR PANIC!");  
  return progressBar(50,HIGH);
}

bool progressBar(int t, int l) {
  int i;
  lcd.setCursor(0,1);
  for(i=0;i<16;i++) {
    lcd.write(4);
    beep(t/25);
    //if(((i-1)%4)==0) beep(t/25);
    delay(t);
    if(digitalRead(btnAPin)==l) break;
  }
  return (i==16);
}

void findHr(void) {
  lcd.print("Please wait...");
  lcd.setCursor(0,0);
  lcd.print("Check heart rate");
  lcd.setCursor(0,1);
  if(getHeartRate()) {
    lcd.print("Measuring...    ");
    delay(MEASUREDELAY);
    digitalWrite(ledPin,LOW);
    lcd.setCursor(0,1);
    lcd.print("Pulse:       OK");
    lcd.write(3);
    lcd.setCursor(7,1);
    lcd.print(heartRate);
    waitForPress();
    hrTimeout=millis();
  }
  else {
    lcd.print("Device not found");      
    digitalWrite(ledPin,LOW);
    delay(2000);
  }
}

void showMessage(void) {
  int i=0,j,k=0,l=0;
  digitalWrite(ledPin,LOW);
  lcd.print("             OK");
  lcd.write(3);
  if(savedText[0]==0) {
    lcd.setCursor(0,0);
    lcd.print("(No new message)");
    waitForPress();
  }
  else {
    for(j=0;j<256;j++)
      if(savedText[j]==0) {
        k=j;
        break;
      }
    while((l++<=((k+16)*4))&&(digitalRead(btnBPin)==LOW)) {
      lcd.setCursor(0,0);
      for(j=-16;j<0;j++) {
        lcd.write((i+j)<0?' ':(i+j)<k?savedText[i+j]:' ');
        if(digitalRead(btnBPin)==HIGH) j=0;
      }
      while((j++<16)&&(digitalRead(btnBPin)==LOW))
        delay(10);
      
      i=(i+1)%(k+16);  
    }
    waitForPress();
  }
}

void sendMessage(void) {
  int item=0;
  
  digitalWrite(ledPin,LOW);
  lcd.write(1);
  lcd.print("CONFIRM   NEXT");
  lcd.write(3);
  lcd.setCursor(0,0);
  lcd.print("(Return to menu)");
  while(digitalRead(btnAPin)==LOW);
  delay(200);
  while((digitalRead(btnAPin)==HIGH)&&((millis()-menuTimeout)<MENUTIMEOUT)) {
    if(digitalRead(btnBPin)==HIGH) {
      menuTimeout=lcdTimeout=millis();
      digitalWrite(ledPin,HIGH);
      beep(10);
      lcd.setCursor(0,0);
      item=(item+1)%(MESSAGES+1);
      
      if(item==0)
        lcd.print("(Return to menu)");
      else {
        lcd.print("\"               ");
        lcd.setCursor(1,0);
        lcd.print(TEXTMESSAGE[item-1]);
        lcd.print("\"");
      }
      
      delay(200);
      digitalWrite(ledPin,LOW);
    }
  }
  if((millis()-menuTimeout)<MENUTIMEOUT) {
    beep(10);
    if(item==0) {
      lcd.setCursor(0,1);
      lcd.print("Not sending. OK");
      lcd.write(3);
    }
    else {
      digitalWrite(ledPin,HIGH);
      lcd.setRGB(0,128,255);
      lcd.setCursor(0,1);
      lcd.print("Sending...      ");
      lcd.setCursor(0,1);
      if(gsmSendSMS(item-1)) lcd.print("Message sent OK");
      else lcd.print("Send failed! OK");
      lcd.write(3);
      digitalWrite(ledPin,LOW);
    }
    waitForPress();
  }
  
}

void findOrBuzzHr(void) {
  lcd.print("Please wait...");
  lcd.setCursor(0,0);
  lcd.print("Looking for band");
  lcd.setCursor(0,1);
  if(hrFound) buzzHrMonitor();
  else findHrMonitor();
  if(hrFound)
    lcd.print("Wristband found!");
  else
    lcd.print("Device not found");      
  digitalWrite(ledPin,LOW);
  delay(2000);
}

bool readMessages(void) {
  bool newMessage=false;
  int i=0,j,n;
  
  while(!newMessage&&!smsQuery&&((n=gsmReadMessage(++i))!=-1)) {
    if(n>0) {
      for(j=0;j<n;j++)
        Serial.write(smsBuffer[j]);
      Serial.println("");
      if(smsBuffer[0]=='?')
        smsQuery=true;
      else {
        for(j=0;j<n;j++)
          savedText[j]=smsBuffer[j];
        savedText[j]=0;
        newMessage=true;
      }
      gsmDeleteMessage(i);
    }            
  }
  return newMessage;
}

void showGnssInfo(void) {
  digitalWrite(ledPin,LOW);
  lcd.clear();
  lcd.write(3);
  lcd.setCursor(0,0);
  lcd.print("               ");
  Serial.print("Lat="); Serial.println(gnss.location.lat(),6);
  Serial.print("Lng="); Serial.println(gnss.location.lng(),6);
  lcd.setCursor(15,0);
  lcd.print(gnss.satellites.value());
  
  lcd.setCursor(0,0);
  lcd.print(gnss.location.rawLat().negative?"-":"+");
  lcd.print(gnss.location.rawLat().deg);
  lcd.print(".");
  lcd.print(gnss.location.rawLat().billionths);
  lcd.print("N");
  
  lcd.setCursor(0,1);
  lcd.print(gnss.location.rawLng().negative?"-":"+");
  lcd.print(gnss.location.rawLng().deg);
  lcd.print(".");
  lcd.print(gnss.location.rawLng().billionths);
  lcd.print("E");
  
  waitForPress();            
}


void waitForPress(void) {
  unsigned long t;
  t=millis();
  while((digitalRead(btnBPin)==LOW)&&((millis()-t)<SAFETYTIMEOUT));
  digitalWrite(ledPin,HIGH);
  beep(10);
  delay(200);
  digitalWrite(ledPin,LOW);
}


void beep(int t) {
  digitalWrite(soundPin,HIGH);
  delay(t);
  digitalWrite(soundPin,LOW);
}

void promptRestart(void) {
  lcd.setRGB(255,0,0);
  lcd.setCursor(0,1);
  lcd.print("System halted.  ");
  while(1);
  /*lcd.print("Press to restart");
  while(digitalRead(btnAPin)==HIGH);
  lcd.setRGB(0,0,0);
  delay(50);
  while(digitalRead(btnAPin)==LOW);
  softwareReset();*/
}

/*void softwareReset(void) {
  SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_HALT_REQ_A;
  SCSS_REG_VAL(SCSS_RSTC) = RSTC_WARM_RESET;
}*/

void alarm(void) {
  int i,j;
  lcd.setCursor(0,0);
  lcd.print("Alarm activated!");
  lcd.setCursor(0,1);
  lcd.print("Press to cancel");
  lcd.write(3);
  
  while(digitalRead(btnBPin)==LOW) {
    for(i=0;i<2;i++) {
      digitalWrite(soundPin,HIGH);
      for(j=0;j<4;j++) {
        lcd.setRGB(255*(i%2),0,255*((i+1)%2));
        delay(40);
        lcd.setRGB(0,0,0);
        delay(120);
      }
      digitalWrite(soundPin,LOW);
      while((j++<40)&&(digitalRead(btnBPin)==LOW))
        delay(10);
      if(digitalRead(btnBPin)==HIGH) i=2;
    }
  }
  waitForPress();
}


// GSM --------------------------------------------------------

bool gsmSendSMS(int n) {
  bool result=false;
  Serial.print("Sending SMS ");
  Serial.println(n);
  Serial1.print("AT+CMGS=\"");
  Serial1.print(MOBILE_NUMBER);
  Serial1.println("\"");
  if(gsmWait()) {
    if(n==REPORT) Serial1.print("Status report");
    if(n==REPORT+1) Serial1.print("WARNING - PANIC pressed");
    if(n==REPORT+2) Serial1.print("WARNING - heart rate");
    if(n==REPORT+3) Serial1.print("WARNING - location");
    if(n==REPORT+4) Serial1.print("WARNING - fall detected");
    delay(500);
    if(n>=REPORT) {
      Serial1.print(": HR ");
        Serial1.print(heartRate);
        Serial1.print("bpm , ");
        Serial1.print(gnssDistance);
        Serial1.print("m from home, location ");
        if(gnss.location.rawLat().negative) Serial1.print("-");
        Serial1.print(gnss.location.rawLat().deg);
        Serial1.print(".");
        Serial1.print(gnss.location.rawLat().billionths);
        Serial1.print("N, ");
        if(gnss.location.rawLng().negative) Serial1.print("-");
        Serial1.print(gnss.location.rawLng().deg);
        Serial1.print(".");
        Serial1.print(gnss.location.rawLng().billionths);
        Serial1.print("E.");        
    }
    else {
      Serial1.print(TEXTMESSAGE[n]);
      Serial1.println(".");
    }
               
    Serial1.println((char)26);
    result=gsmWait();
  }
  else {
    Serial.print("Unable to send SMS");
    Serial1.println((char)26);
  }
  return result;
}

void gsmDeleteMessage(int n) {
  Serial.println("Deleting message");
  Serial1.print("AT+CMGD=");
  Serial1.print(n);
  Serial1.print("\r");
  gsmWait();
  delay(1000);
  gsmWait();
}

int gsmReadMessage(int n) {
  int i,j;
  
  Serial1.print("AT+CMGR="); //read message index=
  Serial1.print(n);
  Serial1.print("\r");
  while(!Serial1.available());
  //delay(100);
  i=0;
  while(Serial1.available()) {
    smsBuffer[i<SMSBUFF?i++:i]=Serial1.read();
    if((i>2)&&(smsBuffer[i-1]=='\n')) break;
    for(j=0;j<10;j++) {
      if(Serial1.available()) break;
      delay(5);
    }
  }
  switch(smsBuffer[2]) {
    case 'O' : 
      Serial.print("Message ");
      Serial.print(n);
      Serial.println(" empty.");
      n=0;        
      break;
    case '+' :
      Serial.print("Message ");
      Serial.print(n);
      Serial.print(" (");
      Serial.print(i);
      Serial.print("): ");
      i=0;
      while(Serial1.available()) {
        smsBuffer[i<SMSBUFF?i++:i]=Serial1.read();
        if((i>2)&&(smsBuffer[i-1]=='\n')) break;
        for(j=0;j<10;j++) {
          if(Serial1.available()) break;
          delay(5);
        }
      }
      smsBuffer[i-1]=0;
      n=i-2;
      break;
    default :
      Serial.print("Stopped at ");
      Serial.print(n);
      Serial.println("");
      n=-1;
  }
  while(Serial1.available()) {
    Serial1.read();
    for(j=0;j<10;j++) {
      if(Serial1.available()) break;
      delay(5);
    }
  }

  return n;
}

int gsmMessages() {
  int i=0;
  char s[SERBUFF];
  Serial1.print("AT+CPMS=\"SM\"\r"); //available/used messages
  delay(100);
  while(Serial1.available())
    s[i<SERBUFF?i++:i]=Serial1.read();

  if(s[i-3]=='K') {
    i=0;
    while(s[i++]!=','&&i<SERBUFF);
    if(i<SERBUFF) {
      if(s[i-3]==' ') s[i-3]='0';
      i=(s[i-3]-'0')*10+(s[i-2]-'0');
      Serial.print(i);
      Serial.println(" messages.");
    }
  }
  else {
    Serial.println("Can't read number of messages!");
    for(i=0;i<SERBUFF;i++)
      Serial.write(s[i]);
    Serial.println("");
    i=0;
  }

  return i;
}

bool gsmWait() {
  unsigned long t;
  bool result=false;
  
  t=millis();
  while((millis()-t)<GSMTIMEOUT) {
    if(Serial1.available())
      if(Serial1.read()=='\n') {
        t=0;
        result=true;
        break;
      }
  }
  if(t!=0)
    Serial.println("Timeout waiting for GSM response!");

  return result;
}


bool gsmInitialise(void) {
  byte a,b=0;
  bool result=false;
  
  Serial1.print("AT\r");
  delay(1000);

  while(Serial1.available()) {
    a=Serial1.read();
    if((a!='\r')&&(a!='\n'))
      b=a;
  }
    
  if(b=='K') {
    result=true;
    Serial.println("OK from GSM module");
  }
  else Serial.println("Bad/no response from GSM module");
  
  if(result) {
    Serial1.print("ATE0\r"); // echo off
    if((result=gsmWait())) {
      Serial1.print("AT+CMGF=1\r"); // SMS in text mode
      if((result=gsmWait())) {
        Serial1.print("AT+CNMI=1,0,0,0,0\r"); // disable notifications
        result=gsmWait();
      }
    }
  }
  
  return result;
}

bool gsmPowerOn(void) {
  bool result=false;
  
  while(Serial1.available())
    Serial1.read();
  Serial1.print("AT\r");
  delay(500);
  
  if(Serial1.available()) {
    while(Serial1.available())
      Serial1.read();
      Serial.println("GSM module already on"); 
    result=true;
  }
  else {
    Serial.println("Starting GSM module");
    digitalWrite(gsmPwrPin,HIGH);
    delay(1200);
    digitalWrite(gsmPwrPin,LOW);
    delay(2800);
    
    if(Serial1.available()) {
      result=true;
      while(Serial1.available())
        Serial1.read();
    }
  }
  return result;    
}



// BLE --------------------------------------------------------

bool findHrMonitor(void) {
  bool found=false;
  unsigned int t;

  BLE.begin();
  BLE.scan();
  t=millis();  

  while((!found)&&((millis()-t)<BLETIMEOUT)) {
    hrMonitor=BLE.available();
    if(hrMonitor)
      if(hrMonitor.localName()==HR_NAME) {
        lcd.print("Found HR band...");
        Serial.println("Found HR monitor");        
        
        if(hrMonitor.connect()) {
          lcd.setCursor(0,1);
          lcd.print("Pairing band... ");       
          Serial.println("Connected to HR monitor");          
          
          if(hrMonitor.discoverAttributes()) {
            Serial.println("Found BLE attributes");          
            
            found=true;
            ctrlCharacteristic=hrMonitor.characteristic(CTRL_CHRC_UUID);
            hrCharacteristic=hrMonitor.characteristic(HR_CHRC_UUID);
            stepCharacteristic=hrMonitor.characteristic(STEP_CHRC_UUID);

            ctrlCharacteristic.writeByte(CTRL_VIBRATE);
            
            hrCharacteristic.setEventHandler(BLEValueUpdated,bleNotify);
            stepCharacteristic.setEventHandler(BLEValueUpdated,bleNotify);
            
            if(stepCharacteristic.subscribe()) Serial.println("Subscribed to step notification");
            if(hrCharacteristic.subscribe()) Serial.println("Subscribed to HR notification");
         }
         hrMonitor.disconnect();
        }      
      }
  }
  BLE.stopScan();
  
  return found;
}

void buzzHrMonitor(void) {
  if(hrMonitor.connect()) {
    Serial.println("Sending vibrate command");      
    ctrlCharacteristic.writeByte(CTRL_NOTIFY);
    delay(500);
    hrMonitor.disconnect();
  }          
  else
    hrFound=false;
}

bool getHeartRate(void) {
  if(hrFound)
    if(hrMonitor.connect()) {
      Serial.println("Starting heart rate measurement");
      ctrlCharacteristic.writeByte(CTRL_HR_START);
      //delay(1000);
      //ctrlCharacteristic.writeByte(CTRL_HR_OPENSERVICE);
      delay(500);
      //if(hrCharacteristic.subscribe()) Serial.println("Subscribed to HR notification");
            
      hrMonitor.disconnect();
    }
  
  return hrFound;
}

