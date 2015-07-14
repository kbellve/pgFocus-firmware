//   (c) 2014
//
//   Karl Bellve
//   Biomedical Imaging Group
//   Molecular Medicine
//   University of Massachusetts Medical School
// 
//   Do not distribute.
//
//   All rights reserved.
//
//   The pgFocus Firmware Source Code is licensed under GPL V3, June 2007.
//
//   http://www.gnu.org/licenses/gpl-3.0.en.html
//
//   You are allowed to modify and use with pgFocus hardware only. 
// 
//   Do Not Distribute the source code. 
//
// Contact: Karl.Bellve@umassmed.edu
//
// http://big.umassmed.edu/wiki/index.php/PgFocus

// include the SPI library:
#include <SPI.h>
#include <Wire.h>
#include <avr/eeprom.h>

#define MAX_DAU 16384
#define MIN_DAU 5000
#define MIDDLE_DAU (MAX_DAU/2)
#define MICRONFOCUSADJUST 0.05 // 50 nanometers
#define DAUPERPIXEL 30        // in DAU units 
#define MICRONPERVOLT 10
#define FOCUSDRIFT 20
#define DAUPERVOLT 1638 // Number of DAU /volt (16384/10 Volts = 1638)
#define FIVE_HUNDRED_NM 82 // DAUPERVOLT/MICRONPERVOLT/2 = 81.9
#define REGRESSIONPOINTS 21
#define EXPOSURE 10000 //(1/100 of a sec)
#define REDLED 1
#define GREENLED 2
#define BLUELED 3
#define ADC_TRIGGER 5
#define ADC_PAUSE 50
#define PREFERENCES 1

struct settings {
  char id[4];
  int mpv;
  double focusAdjust;
  double slope;
  double gain;
  double intercept;
} 
BIG;

int Version = 0;
int Revision = 9;

boolean bStats = false;
boolean bFocus = false;
boolean bPauseFocus = false;
boolean bMedian = false;
boolean bAutoE = true;

int volt = 0;
int light = 1024;
double peak = 128.22;
double error = 100.11;

// DAC
int DAC_SS = A3;
int PD = A2;
int CLR = A1;

// ADC
int ADC_CONVST = A4;
int ADC_BUSY = A5;
int startADC = MIDDLE_DAU, preADC = 0, postADC = 0, diffADC = 0, currentADC = 0;
int skipADC;
double ADC_gain = 1.03458;
unsigned long pauseADC;

// 128 Pixel Line Sensor
int syncPin = 8;
int clockPin = 9;
int dataPin = 10;
int lightVal[128];
int medVal[128];
int maxPixel = 0;
int currentLED = 3;  // current LED to light up
int exposure = EXPOSURE;   // 8333uSec ie 1/120th second
int voltage = MIDDLE_DAU;  // starting voltage. 2048
int micronPerVolt = MICRONPERVOLT;
int maxVal = 1;
int minVal = 1024;
int sensorMax = 0;
int adjustFocus = 0;
boolean bLight = false;

int blinkLED = 0;
boolean bRed = false;
boolean bGreen = false;
boolean bBlue = false;

boolean up = true;

int program = 0;
int flashCount = 0;
int flip = 0;

double focus = 0.0;
double newfocus = 0.0;
double drift;
double dDriftCorrection = 0.5;
double dRegressionPoints[REGRESSIONPOINTS][2];
double dDAUPerPixel = DAUPERPIXEL;
double dFocusAdjust = ((MICRONFOCUSADJUST/MICRONPERVOLT) * DAUPERVOLT) / DAUPERPIXEL;

double dIntercept; 
double dSlope;
double dRR; //residuals
unsigned long FocusCount = 0.0;
unsigned long utime,loopTime, endTime;
unsigned long startTime,time;

uint8_t eePos;  // location of calibration on eeprom

void setup()   {                

  int time_out = 0;
  Serial.begin(57600);

  delay (500);

  //This is blocking...open a serial port to procede...
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
    time_out++;
    if (time_out == 5) break;
    else delay (1000);
  }

  // DAC
  pinMode(DAC_SS, OUTPUT);
  digitalWrite(DAC_SS, HIGH);

  pinMode(PD, OUTPUT);
  digitalWrite(PD, HIGH);

  pinMode(CLR, OUTPUT);
  digitalWrite(CLR, HIGH);

  // ADC
  pinMode(ADC_CONVST, OUTPUT);
  pinMode(ADC_BUSY, INPUT);

  // Line Sensor
  pinMode(clockPin, OUTPUT);
  pinMode(syncPin, OUTPUT);
  pinMode(dataPin, INPUT);


  pinMode(12, OUTPUT); // BLUE
  pinMode(13, OUTPUT); // RED
  pinMode(A0, OUTPUT); // GREEN

  // Turn off main indicator LED // 5V = off
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);   
  digitalWrite(A0, HIGH);

  loadSettings();

  setDAC(MIDDLE_DAU);
  startADC = getADC();

  // clear the dark current
  getCamera(); 

  startTime = millis();

}

void loop()                     
{

  int incomingByte, regressionPoints, maxDAU;
  double newFocus;
  double dDet, dSumX, dSumXX, dSumY, dSumXY;
  double dMeanError, dResiduals;

  endTime = micros();

  preADC = getADC();

  getCamera();

  postADC = getADC();
  diffADC = postADC - preADC;
  currentADC = postADC - startADC;
  if (bFocus == true) {
    //if ((abs(diffADC) >= ADC_TRIGGER) || (abs(currentADC) >= ADC_TRIGGER)) {
    if (abs(diffADC) >= ADC_TRIGGER) {
      pauseADC = millis();
      bPauseFocus = true;
      //Serial.println("Pausing Focus");
    }

  } 
  else startADC = (postADC/2) + (preADC/2);// try to increase ADC zero position confidence while not in focus control


  newfocus = getCenter();

  if (bPauseFocus == true) {
    if ((millis() - pauseADC) > ADC_PAUSE) {
      bPauseFocus = false;
    } 
  }

  if (newfocus == 0) lightRed(true); // can't find peak
  else if (bFocus == false) lightGreen(true); // peak detected but no focus control
  else lightBlue(true); // peak detected but with focus control

  if (bStats) {
    printStats();
    bStats = 0;
  }

  readSerial();

  switch (program) {
  case 1: // FOCUS
    if (newfocus == 0) {
      Serial.println("ERROR: Can't find focus!");
      flashLED(REDLED);
      program = 0;
      break;
    }
    if (focus == 0) {
      if (dDAUPerPixel == DAUPERPIXEL) Serial.println("ERROR: Did you forget to calibrate?");
      startTime = millis();
      focus = newfocus; // set starting value
    }
    else {       
      if (adjustFocus == 2) {
        focus -= dFocusAdjust;
        Serial.print("INFO: Center ");
        Serial.println(focus,4);
      }
      else if (adjustFocus == 1) {
        focus += dFocusAdjust;
        Serial.print("INFO: Center ");
        Serial.println(focus,4);
      }
      adjustFocus = 0;
      if (newfocus > focus) up = 0; // move voltage up
      else up = 1;

      // pixel units        
      if (abs(currentADC) >= ADC_TRIGGER) newfocus = newfocus - (currentADC/dDAUPerPixel);

      drift = (focus - newfocus) * dDriftCorrection;

      // convert to DAU/pixel
      drift *= dDAUPerPixel;       

      if (bFocus == true) {
        if (bPauseFocus == false) {
          if (abs(diffADC) < ADC_TRIGGER) {
            voltage += round(drift);
            if (voltage == MAX_DAU - 1) up = 0;
            else if (voltage == MIN_DAU) up = 1; 
            setDAC(voltage);
          }
        } 
      }
    }
    break;

  case 2: // CALIBRATION
    // Determine scale. Moving one camera pixel equals how many volts?
    if (voltage != MIDDLE_DAU) 
    {
      Serial.println("ERROR: Focus Stabilization must be off before you can calibrate!");
      flashLED(REDLED);
      program = 7;
      break;
    }
    if (getCenter() == 0) 
    {
      Serial.println("ERROR: Can't perform calibration without focus!");
      program = 0;
      break;
    } 
    Serial.println("INFO: Calibrating sensor");

    //
    // linear regression  
    //

    // X should really be DAU, but DAU are large numbers and will result in wrap around, so X = focus, and Y = DAU
    // determine top of voltage range
    maxDAU = regressionPoints = 0;
    dSumX = dSumXX = dSumY = dSumXY = 0.0;

    for (voltage = MIDDLE_DAU; voltage< MAX_DAU; voltage +=  FIVE_HUNDRED_NM) {
      setDAC(voltage);
      delay(500);
      getCamera();
      newfocus = getCenter();
      printStats();
      if (newfocus > 25 && newfocus < 100) {
        maxDAU = voltage;
        Serial.println("INFO: Found a new calibration maximum ");
        printStats();
      }
      else break;
    }

    if (maxDAU == 0) {
      Serial.println("ERROR: failed to find focus on sensor during calibration!");
      voltage = MIDDLE_DAU;
      setDAC(voltage);
      printStats();
      break;
    }

    maxDAU = maxDAU > ((6 * FIVE_HUNDRED_NM) + MIDDLE_DAU) ? ((6 * FIVE_HUNDRED_NM) + MIDDLE_DAU) : maxDAU; 
    for (voltage = maxDAU; voltage > 0; voltage -= FIVE_HUNDRED_NM) { // 1638/20  = 81.90 DAU, or roughly 500 nM steps
      setDAC(voltage);

      delay(500);
      getCamera();
      newfocus = getCenter();
      printStats();

      if (newfocus < 100 && newfocus > 28) {
        dSumX += newfocus;
        dSumXX += newfocus * newfocus;
        dSumXY += newfocus * voltage;
        dSumY += voltage;

        dRegressionPoints[regressionPoints][0] = newfocus;
        dRegressionPoints[regressionPoints][1] = voltage;

        Serial.print("CAL: ");
        Serial.print(voltage);          
        Serial.print(" ");
        Serial.println(newfocus,4);
        regressionPoints++;
      }
      if (newfocus <= 28 || voltage <= (MIDDLE_DAU - (6 * FIVE_HUNDRED_NM))) {
        Serial.print("INFO: found bottom ");
        Serial.println(newfocus);
        break;
      }
    } 

    dDet = (regressionPoints * dSumXX - dSumX * dSumX);
    dSlope = (regressionPoints * dSumXY - dSumX * dSumY) / dDet;
    dIntercept = (dSumXX * dSumY - dSumX * dSumXY)/ dDet;
    Serial.print("SLOPE: ");
    Serial.println(1/dSlope,4);
    Serial.print("INTERCEPT: ");

    Serial.println(dIntercept);
    //Serial.println(-dIntercept/dSlope); // reverse since X and Y is backwards
    //Serial.print("INFO: ");
    //if (dSlope < 0) {

    //}
    //dDAUPerPixel = abs(dSlope);
    dDAUPerPixel = dSlope;
    Serial.print("DAU: "); 
    Serial.println(dDAUPerPixel);  

    Serial.print("VPP: ");
    Serial.println((dDAUPerPixel / DAUPERVOLT ) * micronPerVolt);

    dMeanError = dResiduals = 0.0;
    for (int x = 0; x < regressionPoints; x++) {
      dMeanError += dMeanError + ((dRegressionPoints[x][1] - dSumY/regressionPoints) * (dRegressionPoints[x][1] - dSumY/regressionPoints));
      dResiduals += dResiduals + ((dRegressionPoints[x][1] - dSlope * dRegressionPoints[x][0] - dIntercept) * (dRegressionPoints[x][1] - dSlope * dRegressionPoints[x][0] - dIntercept));
    }

    dRR = 1 - (dResiduals/dMeanError);
    Serial.print("RESIDUALS: ");
    Serial.println(dRR,6);

    dFocusAdjust = ((MICRONFOCUSADJUST/micronPerVolt) * DAUPERVOLT) / dDAUPerPixel;
    Serial.print("INFO: Focus Adjust: ");
    Serial.print(dFocusAdjust,4);
    Serial.println(" pixels");

    Serial.println("INFO: Writing calibration values to eeprom");
    saveSettings();

    Serial.println("INFO: Returning objective to default position");
    voltage = MIDDLE_DAU;
    setDAC(voltage);
    delay (500);


    program = 0;
    break;

  case 4:
    if (up == 1) voltage += 100;
    else voltage -= 100;

    // Constrain is inclusive
    voltage = constrain(voltage,MIN_DAU,MAX_DAU -1);
    if (voltage == MAX_DAU - 1) up = 0;
    if (voltage == MIN_DAU) up = 1;

    Serial.println(voltage,DEC);
    setDAC(voltage);
    break;
  }

  loopTime = micros() - endTime;
}

void printStats() {

  time = millis() - startTime;
  Serial.print("STATS: ");
  Serial.print(time,DEC);
  Serial.print(" ");  
  Serial.print(loopTime,DEC);
  Serial.print(" ");
  Serial.print(minVal,DEC);
  Serial.print(" ");
  Serial.print(maxVal,DEC);  
  Serial.print(" ");     
  Serial.print(voltage,DEC);
  Serial.print(" ");
  Serial.print(newfocus,4);
  Serial.print(" "); 
  Serial.print(dDAUPerPixel,4);
  Serial.print(" ");        
  Serial.print(exposure,DEC);
  Serial.print(" ");        
  Serial.print(currentADC,DEC);
  Serial.print(" ");        
  Serial.print(diffADC,DEC);
  Serial.println();     
}

void readSerial() {

  String Command;
  int incomingByte;
  boolean foundReturn = false;

  if (Serial.available()) {

    Command = "";

    while (foundReturn != true) {
      while (Serial.available() > 0 ) {
        incomingByte = Serial.read();
        if (incomingByte == '\r') foundReturn = true;
        else {
          //if (isDigit(incomingByte)) 
          Command += (char)incomingByte;
        }
      }
    }
      
    Command.toLowerCase();

    if (Command.length() == 1) 
    {    
      incomingByte = (int)Command.charAt(0);
      switch(incomingByte) {
      case 'b':
        bBlue = (bBlue > 0) ? 0 : 1;
        lightBlue(bBlue); 
        break;

      case 'c':
        Serial.println("INFO: Calibration Activated");
        program = 2;
        //bStats = bLight = bFocus = 0;
        bLight = bFocus = 0;
        break;  

      case 'e':
        bAutoE = false;
        break;

      case 'E':
        bAutoE = true;
        break;

      case 'd':
        // Adjust Focus Down
        adjustFocus = 2;
        Serial.println("INFO: Focus Down");
        break;

      case 'f':
        bFocus = (bFocus > 0) ? 0: 1; 
        if (bFocus) {
          Serial.println("INFO: Focus ON");
          program = 1;
          bPauseFocus = false;
          //startADC = getADC();
        } 
        else {
          Serial.println("INFO: Focus OFF");
          program = 0;
          bPauseFocus = false;
          //startADC = MIDDLE_DAU;
        }
        focus = 0;
        voltage = MIDDLE_DAU;
        setDAC(voltage);
        break;

      case 'i': // Identify what we are
        Serial.println("BIG-pgFocus");
        break;
        
      case 'l':
        bLight = (bLight > 0) ? 0: 1;
        if (bLight) { 
          printLight();
        }
        break;

      case 's':
        Serial.println("INFO: Stopping");
        program = 0;
        focus = 0;
        //startADC = MIDDLE_DAU;
        bFocus = false;
        voltage = MIDDLE_DAU;
        setDAC(voltage);
        break; 

      case 't':     
        program = 4;
        up = 1;
        Serial.println("INFO: Testing voltage");
        break;

      case 'u':
        // Adjust Focus Up
        adjustFocus = 1;
        Serial.println("INFO: Focus Up");
        break;  

      case 'v':  
        bStats = true; // print out stats next time around
        break;

      case 'x': //flash
        loadSettings();
        break;

      case 'y': //flash
        saveSettings();
        break;

      default: 
        Serial.println("Unknown command");
        program = 0;
        break; 
      }
    }
    else 
    {

      if (Command.startsWith("voltage")) {
        if (Command.length() == 7) {
          double temp = voltage;
          temp = ((temp/MAX_DAU) * 10) - 5;
          Serial.print("VOLTAGE: ");
          Serial.println(temp,2);
        } 
        else {
          Command = Command.substring(7);
          double temp = StringtoDouble(Command);
          if (temp >= -5 && temp <= 5) {
            voltage = ((temp + 5)/10) * MAX_DAU;
            setDAC(voltage);
            Serial.println("OK");
          } 
          else Serial.println("ERROR: Out of Range");
        }
      }

      else if (Command.startsWith("version")) {
        if (Command.length() == 7) {
          Serial.print("VERSION: ");
          Serial.print(Version);
          Serial.print(".");
          Serial.println(Revision);
        }
      }
      else if (Command.startsWith("offset")) { // match uManager terminology of using offset instead of focus
        if (Command.length() == 6) {
          Serial.print("OFFSET: ");
          Serial.println(focus,2);
        }
        else {
          Command = Command.substring(6);
          double temp = StringtoDouble(Command);
          if (temp >=0 && temp <128) {
            focus = temp;
            Serial.println("OK");
          } 
          else Serial.println("ERROR: Out of Range");
        }
      }

      else if (Command.startsWith("slope")) {
        if (Command.length() == 5) {
          Serial.print("SLOPE: ");
          Serial.println(dDAUPerPixel,2);
        }
      }

      else if (Command.startsWith("step")) {
        if (Command.length() == 4) {
          Serial.print("STEP: ");
          Serial.println(dDriftCorrection,2);
        }
        else {
          Command = Command.substring(4);
          double temp = StringtoDouble(Command);
          if (temp > 0 && temp <= 1.5) {
            dDriftCorrection = temp;
            Serial.println("OK");
          } 
          else Serial.println("ERROR: Out of Range");
        }
      }

      else if (Command.startsWith("drift")) {
        Serial.println(drift * 2);
      }

      else if (Command.startsWith("time")) {
        Serial.println(endTime);
      }

      else if (Command.startsWith("mpv")) {
        if (Command.length() == 3) {
          Serial.print("MPV: ");
          Serial.println(micronPerVolt);
        }
        else {
          Command = Command.substring(3);
          int temp = StringtoInt(Command);
          if (temp > 0) {
            micronPerVolt = temp;
            Serial.println("OK");
          } 
          else Serial.println("ERROR: Out of Range");
        }
      }

      else if (Command.startsWith("exposure")) {
        if (Command.length() == 8) {
          Serial.print("EXPOSURE: ");
          Serial.println(exposure);
        }
        else {
          Command = Command.substring(8);
          int temp = StringtoInt(Command);
          if (temp > 0) {
            exposure = temp;
            Serial.println("OK");
          } 
          else Serial.println("ERROR: Out of Range");
        }
      }
      else if (Command.startsWith("gain")) {
        if (Command.length() == 4) {
          Serial.print("GAIN: ");
          Serial.println(ADC_gain,4);
        }
        else {
          Command = Command.substring(4);
          int temp = StringtoDouble(Command);
          if (temp > 0) {
            ADC_gain = temp;
            Serial.println("OK");
          } 
          else Serial.println("ERROR: Out of Range");
        }
      }
      else if (Command.startsWith("intercept")) {
        if (Command.length() == 9) {
          Serial.print("INTERCEPT: ");
          Serial.println(dIntercept,4);
        }
      }
      else if (Command.startsWith("residuals")) {
        if (Command.length() == 9) {
          Serial.print("RESIDUALS: ");
          Serial.println(dRR,6);
        }
      }
      else {
        Serial.println("ERROR: Sorry, I didn't understand the command!");
        
      }
    }
  }
}


void flashLED(int led)
{
  blinkLED = led;

  if (led == 0) {
    flashCount = 0;
    bGreen = bRed = bBlue = false;
    return;
  } 
  else {
    if (flashCount > 10000) {
      switch (led)
      {
      case 1:
        bRed = true;
        break;

      case 2:  
        bGreen = true;
        break;

      case 3:   
        bBlue = true;
        break;
      }
      if (flashCount > 20000) flashCount = 0;
    } 
    else {
      bGreen = bRed = bBlue = false;
    }
    flashCount++;
  }

  lightRed(bRed);
  lightRed(bGreen);
  lightRed(bBlue);
}

void lightRed(boolean on)
{       
  if (on) {
    digitalWrite(13, LOW);
    bGreen = bBlue = false;
    lightGreen(bGreen);
    lightBlue(bBlue);
  }
  else  digitalWrite(13, HIGH);


}

void lightBlue(boolean on)
{
  if (on) {
    digitalWrite(12, LOW);
    bRed = bGreen = false;
    lightRed(bRed);
    lightGreen(bGreen);
  }
  else digitalWrite(12, HIGH);


}

void lightGreen(boolean on)
{ 
  if (on) {
    digitalWrite(A0, LOW);
    bRed = bBlue = false;
    lightRed(bRed);
    lightBlue(bBlue);
  }
  else digitalWrite(A0, HIGH);


}


int getADC() 
{ 

  byte lVin = 0;
  byte hVin = 0;
  int volt2;

  SPI.begin();

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1); // Maybe use SPI_MODE0
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  digitalWrite(ADC_CONVST, HIGH);
  delayMicroseconds(1);
  digitalWrite(ADC_CONVST, LOW);
  delayMicroseconds(5);

  hVin = SPI.transfer(0); 
  lVin = SPI.transfer(0); 

  digitalWrite(ADC_CONVST, HIGH);  

  volt2  = word(hVin,lVin); 

  // The ADC outputs two's complement
  // Check if the MSB (bit 5) of hVin is high, which means the number is negative and the bits should be flipped, plus 1 added)
  if ((hVin & (1 << 5)) != 0) { 
    volt2 = (~volt2 & 0x3FFF ) + 1; 
    volt2 *= -1; 
    //volt3 |= (1<< 15);
    //volt3 |= (1<< 16);
    //volt3 = ~volt3 + 1;
    //volt3 *= -1;
  } 

  //Serial.print("ADC: ");
  //Serial.println(volt2);

  SPI.end();  
  return(ADC_gain * volt2);

}

void setDAC(int DAU) 
{ 
  // 14bit +/- 5Volt

  // Constrain is inclusive 
  DAU = constrain(DAU,MIN_DAU,MAX_DAU - 1);          

  byte hDAU = highByte(DAU);
  byte lDAU = lowByte(DAU);


  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE2); // maybe SPI_MODE1 if you want clock to be low?
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  delay(1);

  digitalWrite(DAC_SS, LOW);

  SPI.transfer(hDAU);
  SPI.transfer(lDAU); 

  delay(1);
  digitalWrite(DAC_SS, HIGH);

  SPI.end();

}

double getCenter()
{
  double Val = 0, Sum = 0;
  int i = 0, start = 0,halfVal = 0;
  int a,b,c, middle;

  maxVal = 0;
  minVal = 1023;
  sensorMax = 0;

  for (int j = 0; j < 128; j++)
  {
    if (maxVal < lightVal[j]) maxVal = lightVal[j];
    if (lightVal[j] < minVal) minVal = lightVal[j];     
  }

  halfVal = maxVal/2;

  if (halfVal < minVal) {
    //Serial.println("ERROR: Can't find focus, not enough light!"); 
    return 0;
  }
  else {
    i = 0;
    for (int j = 0; j < 128; j++)
    {
      if (lightVal[j] > halfVal)
      {  
        if (i == 0) start = j;
        i++;
        Sum += (lightVal[j] - halfVal);
        Val += (lightVal[j] - halfVal) * i;
        if (Val < 0) {
          //Serial.println("ERROR: Can't find focus, camera too saturated!"); 
          return 0; // rolled over
        }
      }
      if (lightVal[j] == 1023) sensorMax++;
    }
  }

  if (bAutoE) {
    if (sensorMax) {
      if (exposure > (EXPOSURE/20)) {
        exposure -= sensorMax * 10;
        if (exposure < (EXPOSURE/20)) exposure = EXPOSURE/20;
      } 
    } 
    else {
      if (maxVal < 512 && exposure < (EXPOSURE + 10)) {   
        exposure += 10; 
        if (exposure > EXPOSURE) exposure = EXPOSURE;
      }
    } 
  }

  double dCenter = (double)((Val/Sum)) + start - 1; // subtract 1 to bring it back to zero indexed

  return (dCenter);

}


//
// getCamera() function is based on code written by Kevin Gordon and does not fall under my copyright. 
// Karl Bellve
//
void getCamera()
{

  // Clear the Camera
  digitalWrite(clockPin, LOW);
  pinMode(syncPin, OUTPUT);
  digitalWrite(syncPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(syncPin, LOW);
  utime = micros();
  digitalWrite(clockPin, LOW);
  for (int j = 0; j < 128; j++)
  {
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }

  // Expose
  delayMicroseconds(exposure);
  digitalWrite(syncPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(syncPin, LOW);
  utime = micros() - utime;
  digitalWrite(clockPin, LOW);

  // Read Pixels
  for (int j = 0; j < 128; j++)
  {
    delayMicroseconds(20);
    lightVal[j] = analogRead(dataPin);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
  delayMicroseconds(20);

  //if (bMedian) medianFilter();

}

double StringtoDouble(String Command) {

  Command.trim();

  if (Command.length() > 0) {
    char buffer[Command.length() + 1];

    Command.toCharArray(buffer,sizeof(buffer));
    return (atof(buffer));
  }

  return (0);
}
int StringtoInt(String Command) {

  Command.trim();

  if (Command.length() > 0) {
    char buffer[Command.length() + 1];

    Command.toCharArray(buffer,sizeof(buffer));
    return (atoi(buffer));
  }

  return (0);
}

void printLight() {
  if (program != 1) { // not needed if focus stabilization is currently running
    getCamera();
    getCenter();
  }
  time = millis() - startTime;
  Serial.print("LIGHT: ");
  Serial.print(time);
  for (int j = 0; j < 128; j++)
  {
    Serial.print(" ");
    Serial.print(lightVal[j],DEC);  
  }
  Serial.println();
  bLight = 0;
}

char *ftoa(char *a, double f, int precision)
{
  long p[] = {
    0,10,100,1000,10000,100000,1000000,10000000,100000000  };

  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

void loadSettings() {

  Serial.println("INFO: Loading preferences");
  // read past voltage per pixel calibration
  if (sizeof(BIG) > 512) Serial.println("ERROR: Settings too large for eprom");
  eeprom_read_block((void*)&BIG,(void*)0,sizeof(BIG));
  if (BIG.id[0] == 'B') {
    if (BIG.id[1] == 'I') {
      if (BIG.id[2] == 'G') {
        if (BIG.id[3] == 1) {
          Serial.println("INFO: Found preferences");

          if (BIG.focusAdjust <= MICRONFOCUSADJUST) dFocusAdjust = MICRONFOCUSADJUST;
          else dFocusAdjust = BIG.focusAdjust; 
          Serial.print("INFO: Focus Adjust: ");
          Serial.print(dFocusAdjust,4);
          Serial.println(" pixels");

          micronPerVolt = BIG.mpv;
          Serial.print("MPV: ");
          Serial.println(micronPerVolt); 

          dDAUPerPixel = BIG.slope;
          Serial.print("SLOPE: ");
          Serial.println(dDAUPerPixel);  

          ADC_gain = BIG.gain;
          Serial.print("GAIN: ");
          Serial.println( ADC_gain); 

          dIntercept = BIG.intercept;
          Serial.print("INTERCEPT: ");
          Serial.println(dIntercept); 
        }

        return;
      }
    }
  } 
  Serial.println("ERROR: Failed to find settings");
}

void saveSettings() {

  Serial.println("INFO: Saving preferences");
  BIG.id[0] = 'B';
  BIG.id[1] = 'I';
  BIG.id[2] = 'G';
  BIG.id[3] =  PREFERENCES; // preferences version
  BIG.focusAdjust = dFocusAdjust;
  BIG.slope = dDAUPerPixel;
  BIG.gain = ADC_gain;
  BIG.intercept = dIntercept;
  BIG.mpv = micronPerVolt;

  eeprom_write_block((void*)&BIG,(void *)0,sizeof(BIG));
}


