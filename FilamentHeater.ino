/*Code designed by Sujay Alaspure in SA Lab */

#include <Wire.h>
#include "DisplaySupport.h"
#include <MovingAverage.h>


const int sensor1=A0; // Assigning analog pin A5 to variable 'sensor'
float tempc1; //variable to store temperature in degree Celsius
float tempf1; //variable to store temperature in Fahreinheit
float vout1; //temporary variable to hold sensor reading

const int sensor2=A1; // Assigning analog pin A5 to variable 'sensor'
float tempc2; //variable to store temperature in degree Celsius
float tempf2; //variable to store temperature in Fahreinheit
float vout2; //temporary variable to hold sensor reading

const int tempPotentiometer=A2; // Assigning analog pin A5 to variable 'sensor'
float setTemp; //variable to store temperature in Fahreinheit
float vout3; //temporary variable to hold sensor reading
float initSetTemp = 0;
float tempsArray[10] = {0};
int PWMSteps[3] = {5, 60, 200};

enum Stages {
  bumpTemp,
  equilizeTemp,
  overTemp
};

Stages currentStage = bumpTemp;
int localStep = 1;
float localSP = 0;
int localPWMStep = 0;

long lastSensorsread = 0;
long tempEqStartCycle = 0;
long tempOvrStartCycle = 0;
int lastState = 0;
MovingAverage<float> HeaterTemp(20, 25);
MovingAverage<float> FarSensorTemp(20, 25);
MovingAverage<float> SetTemp(20, 45);


void setup() 
{
pinMode(sensor1,INPUT); // Configuring sensor pin as input
pinMode(sensor2,INPUT); // Configuring sensor pin as input
pinMode(tempPotentiometer,INPUT);
pinMode(8,OUTPUT); // Heater 1
pinMode(A9,OUTPUT); // Heater 2
pinMode(A10,OUTPUT); //FAN
analogWrite(A10,250);
delay(5000);
analogWrite(A10,5);
analogReadResolution(16);
Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forevert
  }
  ClearScreen();
  drawText(0,1,"Sensor 1: ");
  drawText(0,3,"Sensor 2: ");
  drawText(0,4,"Set C: ");

  for (int i = 0 ; i < 20 ; i++)
  {
    vout1=analogRead(sensor1); //Reading the value from sensor
    tempc1=((vout1*3.3)/65535-0.5)*100;
    HeaterTemp.push(tempc1);

  
  // Far sensor
    vout2=analogRead(sensor2); //Reading the value from sensor
    tempc2=((vout2*3.3)/65535-0.5)*100;
    FarSensorTemp.push(tempc2);
    
    vout3=analogRead(tempPotentiometer); //Reading the value from potentiometer
    setTemp=((vout3*85.0)/65535.0);
    SetTemp.push(setTemp);
    delay(250);
  }
  initSetTemp = SetTemp.get();
  float delta = (initSetTemp - FarSensorTemp.get());
  tempsArray[0] = FarSensorTemp.get();
  tempsArray[1] = tempsArray[0] + 0.33 * delta;
  tempsArray[2] = tempsArray[1] + 0.25 * delta;
  tempsArray[3] = tempsArray[2] + 0.16 * delta;
  tempsArray[4] = tempsArray[3] + 0.12 * delta;
  tempsArray[5] = tempsArray[4] + 0.10 * delta;
  tempsArray[6] = tempsArray[5] + 0.04 * delta;

  localSP = tempsArray[localStep];
}

void loop() 
{
  readSensors();

  ClearDigits(11, 1, 6);
  ClearDigits(11, 3, 6);
  drawNumber(11, 1, HeaterTemp.get());
  drawNumber(11, 3, FarSensorTemp.get());
  
  if ( (HeaterTemp.get() > 0.9 * localSP) &&  (HeaterTemp.get() < 1.1 * localSP) && (FarSensorTemp.get() > 0.9 * localSP) && (lastState != 2) && (currentStage != overTemp))
  {
    digitalWrite(A9, LOW);
    analogWrite(A10,128);
    ClearDigits(0, 2, 14);
    drawText(0,2,"Heaters OFF");
    lastState = 2;
  }
  else if ( (currentStage == bumpTemp) && (HeaterTemp.get() > localSP) )
  {
    currentStage = equilizeTemp;
    tempEqStartCycle = millis();
  }
  else if ( (HeaterTemp.get() < 1.25 * localSP) && (FarSensorTemp.get() <= 0.9 * localSP) && (lastState != 1) && (currentStage == bumpTemp))
  {
    digitalWrite(A9, HIGH);
    analogWrite(A10,60);
    ClearDigits(0, 2, 14);
    drawText(0,2,"Heaters ON");
    lastState = 1;
  }
  else if ((HeaterTemp.get() >= 1.25 * initSetTemp) && (lastState != 3))
  {
    digitalWrite(A9, LOW);
    analogWrite(A10,250);
    ClearDigits(0, 2, 14);
    drawText(0,2,"Heaters OFF");
    lastState = 3;
  }
  else if ( (currentStage == equilizeTemp) && (lastState != 5) )
  {
      ClearDigits(0, 2, 14);
      drawText(0,2,"EqTemp ON");
      analogWrite(A10,250);
      tempEqStartCycle = millis();
    while ( (FarSensorTemp.get() < localSP - 3))
    {
      readSensors();
      drawTemps();
      if (millis() - tempEqStartCycle >= 1000 * 60 * 1)
      {
        localPWMStep++;
        if (localPWMStep >= 3)
        {
          localPWMStep = 0; 
        }
        analogWrite(A10,PWMSteps[localPWMStep]);
        if ( ((localPWMStep % 2) == 0) || (localSP >= 55.0) )
        {
          digitalWrite(A9, HIGH);
          ClearDigits(0, 2, 14);
          drawText(0,2,"EqTemp ON");
        }
        else
        {
          digitalWrite(A9, LOW);
          ClearDigits(0, 2, 14);
          drawText(0,2,"EqTemp OFF");
        }
        tempEqStartCycle = millis();
      }
    }
    localStep++;
    if (localStep >= 6)
    {
      localStep = 6;
    }
    localSP = tempsArray[localStep];
    currentStage = bumpTemp;
    lastState = 5;
  }
  else if ( ((currentStage == bumpTemp) && ( (HeaterTemp.get() - FarSensorTemp.get() >= 10) || (HeaterTemp.get() - localSP >= 5) )) || (currentStage == overTemp))
  {
    lastState = 4;
    currentStage = overTemp;
    if (millis() - tempOvrStartCycle >= 1000 * 30 * (1 + localPWMStep ))
    {
      if (localPWMStep >= 3)
      {
        localPWMStep = 0; 
      }
      analogWrite(A10,PWMSteps[localPWMStep]);
      if ( (localPWMStep % 2) == 0)
      {
        digitalWrite(A9, HIGH);
        ClearDigits(0, 2, 14);
        drawText(0,2,"OvrTemp ON");
      }
      else
      {
        digitalWrite(A9, LOW);
        ClearDigits(0, 2, 14);
        drawText(0,2,"OvrTemp OFF");
      }
      localPWMStep++;
      tempOvrStartCycle = millis();
    }
    if ( ((HeaterTemp.get() - FarSensorTemp.get() <= 5) || (localSP - FarSensorTemp.get() <= 1)) && (currentStage == overTemp))
    {
      currentStage = bumpTemp;
    }
      
  }
  ClearDigits(7, 4, 16);
  drawNumber(7, 4, SetTemp.get());
  drawNumber(13, 4, localSP);

  if (fabs(SetTemp.get() - initSetTemp) >= 5)
  {
    initSetTemp = SetTemp.get();
    float delta = (initSetTemp - FarSensorTemp.get());
    tempsArray[0] = FarSensorTemp.get();
    tempsArray[1] = tempsArray[0] + 0.33 * delta;
    tempsArray[2] = tempsArray[1] + 0.25 * delta;
    tempsArray[3] = tempsArray[2] + 0.16 * delta;
    tempsArray[4] = tempsArray[3] + 0.12 * delta;
    tempsArray[5] = tempsArray[4] + 0.10 * delta;
    tempsArray[6] = tempsArray[5] + 0.04 * delta;

    localSP = tempsArray[localStep];
  }
}

void readSensors(void)
{
  if (millis() - lastSensorsread >= 250)
  {
    vout1=analogRead(sensor1); //Reading the value from sensor
    tempc1=((vout1*3.3)/65535-0.5)*100;
    HeaterTemp.push(tempc1);

  
  // Far sensor
    vout2=analogRead(sensor2); //Reading the value from sensor
    tempc2=((vout2*3.3)/65535-0.5)*100;
    FarSensorTemp.push(tempc2);
    
    vout3=analogRead(tempPotentiometer); //Reading the value from potentiometer
    setTemp=((vout3*85.0)/65535.0);
    SetTemp.push(setTemp);
    lastSensorsread = millis();
  }
}

void drawTemps(void)
{
  ClearDigits(11, 1, 6);
  ClearDigits(11, 3, 6);
  drawNumber(11, 1, HeaterTemp.get());
  drawNumber(11, 3, FarSensorTemp.get());
}
