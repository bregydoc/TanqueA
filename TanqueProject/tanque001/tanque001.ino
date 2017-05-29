//tanque001.ino

#include <Wire.h>
#include <math.h>
#include "Adafruit_FONA.h"

#define DEBUG_MODE  

#define fonaSerial Serial1
             

#define FONA_RST 4            

#define heightFilterCorrect 5.0

#define ultrasonicAnalogPin 17
#define ledDebug 13

#define MPU 0x68

#define A_R 16384.0
#define G_R 131.0
 
#define ratioDegToRad  57.295779

#define Msize 70.0
#define Nsize 58.5
#define Psize 63.0


////////////////////////////////////////
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

float speedKmh;

////////////////////////////////////////

bool firstStart;
int currentState;
float currentHeight;

////////////////////////////////////////
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

float Acc[2];
float Gy[2];
float Angle[2];

////////////////////////////////////////
float currentVolume;
////////////////////////////////////////
bool ledDebugState;

////////////////////////////////////////
///////  FUNCTIONS  PROTOTYPES  ////////
void initAllConnectionsAndVariables();
void checkNetworkStatus();
void activateGpsAndGprs();
float simpleFilter(float lastValue, float currentValue);
void getGPRS();
void initMPU6050();
void getCurrentHeight();
void getCurrentvolume();
void getCurrentFuel(bool type);

void setup() {
  initAllConnectionsAndVariables();
}

void loop() {
  for (int i=0;i<5;i++) {
    currentState = i;
    stateMachineStep();
  }
  delay(100);
}





void initAllConnectionsAndVariables() {

  ledDebugState = true;
    firstStart = true;
    currentHeight = -1;
    currentState = -1;
    currentVolume = 0;

    speedKmh = 0; 

    initMPU6050();
  
    activateGpsAndGprs();

    pinMode(ultrasonicAnalogPin, INPUT);

    pinMode(ledDebug, OUTPUT);


    fonaSerial.begin(4800);
    fona.begin(fonaSerial);
    fona.enableGPS(true);


}


void initMPU6050() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void activateGpsAndGprs() {
  //fona.enableGPS(true);
  //fona.enableGPRS(true);
}



float simpleFilter(float lastValue, float currentValue) {
  float delta = currentValue - lastValue;
  
    if (delta < 0) {
      delta = delta * -1;
    }

    if (delta > heightFilterCorrect) {
      return lastValue;
    }else{
      return currentValue;
    }
}

void getCurrentHeight() {
  /*
  if (firstStart) {
    
  }
  */

  #ifdef DEBUG_MODE
    Serial.println("Getting current height");
    #endif

    if (millis()<10000) {
      currentHeight = -1.;
    }else{
      int read = analogRead(ultrasonicAnalogPin);

      float lastHeight = currentHeight;

      float dist = 0.5758*read;
      currentHeight = 103. - dist;

      currentHeight = simpleFilter(lastHeight, currentHeight);
  }
}



void getAllAngles() {
    /*
    const forCalc = 57.295791; //180/3.141592

    arx = forCalc*atan(ax / sqrt(square(ay, 2) + square(az, 2))); 
    ary = forCalc*atan(ay / sqrt(square(ax, 2) + square(az, 2)));
    arz = forCalc*atan(sqrt(square(ay) + square(ax)) / az);

    rx = (0.1 * arx) + (0.9 * grx);
    ry = (0.1 * ary) + (0.9 * gry);
    rz = (0.1 * arz) + (0.9 * grz);

    rx = (0.96 * arx) + (0.04 * grx);
    ry = (0.96 * ary) + (0.04 * gry);
    rz = (0.96 * arz) + (0.04 * grz);
    */
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();
    AcZ = Wire.read()<<8|Wire.read();
 

    Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R), 2)+pow((AcZ/A_R), 2)))*ratioDegToRad; //*RAD_TO_DEG; // 
    Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R), 2)+pow((AcZ/A_R), 2)))*ratioDegToRad; //*RAD_TO_DEG;
 
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,4,true);
    GyX = Wire.read()<<8|Wire.read();
    GyY = Wire.read()<<8|Wire.read();
 
    Gy[0] = GyX/G_R;
    Gy[1] = GyY/G_R;
 
    Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
    Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
 
}

void getCurrentFuel(bool type) { 
  if (type) {
    currentVolume = Msize*currentHeight*Nsize;
  }else {

    float part1 = (pow(currentHeight, 2) * cos(Angle[0]))/2 + pow(Msize, 2)/8*cos(Angle[0]) + currentHeight*Msize/2;
    float part2 = (pow(currentHeight, 2) * cos(Angle[1]))/2 + pow(Nsize, 2)/8*cos(Angle[1]) + currentHeight*Nsize/2;

    currentVolume = (part1*part2*2*currentHeight*cos(Angle[0]))/(-1*(2*currentHeight*cos(Angle[0])-Msize)*currentHeight);
  }


} 



void getSpeed() {
  float latitude, longitude, heading, altitude, speed_kph;

  bool gpsSuccess = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  if (gpsSuccess) {
    speedKmh = speed_kph;
  }

} 


void stateMachineStep() {
  if (currentState == 0) {
      getCurrentHeight();
    }else if (currentState == 1) {
      getAllAngles();
    }else if (currentState == 2) {
      getCurrentFuel(true);
    }else if (currentState == 3) {
      getSpeed();
    }else if (currentState == 4) {
      digitalWrite(ledDebug, ledDebugState xor true);
      delay(10);
    }
}




