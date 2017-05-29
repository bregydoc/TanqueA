//tanque001.ino

#include <Wire.h>
#include <math.h>
#include "Adafruit_FONA.h"

#include <SD.h>
#include <SPI.h>


#define DEBUG_MODE  

#define fonaSerial Serial1
             

#define FONA_RST 4            

#define heightFilterCorrect 5.0

#define ultrasonicAnalogPin 17

#define MPU 0x68

#define A_R 16384.0
#define G_R 131.0
 
#define ratioDegToRad  57.295779

#define Msize 70.0
#define Nsize 58.5
#define Psize 63.0

//////////// STATE MACHINE /////////////
#define globalClockPeriod 100
////////////////////////////////////////

//////////////// SD CARD ///////////////
#define chipSelect 15
////////////////////////////////////////

#define consumptionThreshold 100.0  
#define numberMessage "0051957821858"
////////////////////////////////////////
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

float speedKmh;
float currentLatitude, currentLongitude;

////////////////////////////////////////

bool firstStart;
int currentState;
int ticksCount;
////////////////////////////////////////

float currentHeight;
////////////////////////////////////////
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

float Acc[2];
float Gy[2];
float Angle[2];

////////////////////////////////////////
float currentVolume;

float currentConsumption;
float accumConsumption;
////////////////////////////////////////
bool ledDebugState;
////////////////////////////////////////
int alertState;
//------------------------------
//   alertState |      mean
//------------------------------
//       0     | Nothing, all ok
//       1     | Alert shotted
//------------------------------
////////////////////////////////////////

///////  FUNCTIONS  PROTOTYPES  ////////
void initAllConnectionsAndVariables();
void checkNetworkStatus();
float simpleFilter(float lastValue, float currentValue);
void getGPRS();
void initMPU6050();
void getCurrentHeight();
void getCurrentvolume();
void getCurrentFuel(bool type);
bool sendSMSLowLevel();
void saveCurrentStateintoSD();
void refreshConsumption(bool param1);
void sendAlertToBoss();

void setup() {
    #ifdef DEBUG_MODE
    Serial.begin(9600);
    #endif

    initAllConnectionsAndVariables();
    
    firstStart = true;

    for (int i=0;i<6;i++) {
      currentState = i;
      stateMachineStep();
    }

    firstStart = false;
}


void loop() {
    for (int i=0;i<6;i++) {
        currentState = i;
        stateMachineStep();
    }

    delay(globalClockPeriod);

    ticksCount += 1;
    if (ticksCount>10000) {
        ticksCount = 0;
    }
}





void initAllConnectionsAndVariables() {

    ledDebugState = true;
    currentHeight = -1;
    currentState = -1;
    currentVolume = 0;

    speedKmh = 0;
    currentLatitude = 0;
    currentLongitude = 0;

    currentConsumption = 0;
    accumConsumption = 0;
    
    alertState = 0;

    ticksCount = 0;

    initMPU6050();
  
    pinMode(ultrasonicAnalogPin, INPUT);

    fonaSerial.begin(4800);
    
    fona.begin(fonaSerial);

    fona.enableGPS(true);
    fona.enableGPRS(true);

    SPI.setMISO(12);
    SPI.setMOSI(11);  
    SPI.setSCK(13); 

    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
    }else{
        Serial.println("card initialized.");
    }

}


void initMPU6050() {
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
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
    //01000110100000000 or 10101101 -> 01000110110101101 

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

      float part1 = (pow(currentHeight, 2)*cos(Angle[0]))/2 + pow(Msize, 2)/8*cos(Angle[0]) + currentHeight*Msize/2;
      float part2 = (pow(currentHeight, 2)*cos(Angle[1]))/2 + pow(Nsize, 2)/8*cos(Angle[1]) + currentHeight*Nsize/2;

      currentVolume = (part1*part2*2*currentHeight*cos(Angle[0]))/(-1*(2*currentHeight*cos(Angle[0])-Msize)*currentHeight);
  }


} 



void getSpeed() {
    float latitude, longitude, heading, altitude, speed_kph;

    bool gpsSuccess = fona.getGPS(&latitude, &currentLongitude, &speed_kph, &heading, &altitude);

    if (gpsSuccess) {
        currentLatitude = latitude;
        currentLongitude = longitude;

        speedKmh = speed_kph;
    }

    if (currentLatitude == 0.0 || currentLongitude == 0.0) {
         if (fona.getNetworkStatus() == 1) {
            bool gsmlocSuccess = fona.getGSMLoc(&latitude, &longitude);
            
            if (gsmlocSuccess) {
                currentLatitude = latitude;
                currentLongitude = longitude;
                 speedKmh = speed_kph;

            }else{
                #ifdef DEBUG_MODE
                Serial.println("GSM location failed...");
                Serial.println(F("Disabling GPRS"));
                #endif
                fona.enableGPRS(false);
                #ifdef DEBUG_MODE
                Serial.println(F("Enabling GPRS"));
                if (!fona.enableGPRS(true)) {
                    Serial.println(F("Failed to turn GPRS on"));  
                }
                #endif

            }
        }
    }

   

} 


void stateMachineStep() {
    if (currentState == 0) {
        #ifdef DEBUG_MODE
        Serial.println("Into state s0, refreshing height, angles, fuel(volume) and speed");
        #endif

        getCurrentHeight();
        getAllAngles();
        getCurrentFuel(true);
        getSpeed();


        
    }else if (currentState == 1) {
        #ifdef DEBUG_MODE
        Serial.println("Into state s1");
        #endif

        if (alertState != 0) {
            #ifdef DEBUG_MODE
            Serial.println("Alert Activated");
            #endif

            if (alertState == 1) {
                 #ifdef DEBUG_MODE
                Serial.println("Alert type 001");
                #endif

                sendAlertToBoss();

                alertState = 0;
            }else if(alertState == 2) {
              //...
              #ifdef DEBUG_MODE
              Serial.println("Alert type 002");
              #endif
            }
            //...
        }

    
    }else if (currentState == 2) {
        #ifdef DEBUG_MODE
        Serial.println("Into state s2");
        Serial.println("Refreshing the consumption...");

        #endif
        refreshConsumption(true);

    }else if (currentState == 3) {
        #ifdef DEBUG_MODE
        Serial.println("Into state s3");
        #endif


    }else if (currentState == 4) {
        #ifdef DEBUG_MODE
        Serial.println("Into state s4");
        #endif

        //digitalWrite(ledDebug, ledDebugState xor true);
        //delay(10);

    }else if(currentState == 5) {
        #ifdef DEBUG_MODE
        Serial.println("--------- Save the package into SD ---------");
        #endif
        if ((ticksCount*globalClockPeriod) % 1000 == 0) {
            saveCurrentStateintoSD();
        }
    }


    #ifdef DEBUG_MODE
    Serial.print("Raw Height: "); Serial.println(analogRead(ultrasonicAnalogPin));

    Serial.print("Height: "); Serial.println(currentHeight);
    Serial.print("Volume: "); Serial.println(currentVolume);
    Serial.print("Speed: "); Serial.println(speedKmh);
    Serial.print("Angles: "); Serial.print(Angle[0]); Serial.print(", "); Serial.println(Angle[1]);
    Serial.print("Consumption: "); Serial.println(currentConsumption);

    #endif
}

void saveCurrentStateintoSD() {
    String header = "rawHeight, Height, GPS:Latitude, GPS:Longitude, GPS:Speed, Angles:0, Angle:1, Volume, Consumption";
    String data = "";

    int raw = analogRead(ultrasonicAnalogPin);

    data += String(raw);
    data += ",";
    data += String(currentHeight);
    data += ",";
    data += String(currentLatitude);
    data += ",";
    data += String(currentLongitude);
    data += ",";
    data += String(speedKmh);
    data += ",";
    data += String(Angle[0]);
    data += ",";
    data += String(Angle[1]);
    data += ",";
    data += String(currentVolume);
    data += ",";
    data += String(currentConsumption);

    File dataFile = SD.open("Datalog.csv", FILE_WRITE);

    if (dataFile) {
        dataFile.println(data);
        dataFile.close();
        #ifdef DEBUG_MODE
        Serial.println(data);
        #endif
    }  
    else {
        #ifdef DEBUG_MODE
        Serial.println("error opening Datalog.csv");
        #endif
    } 

}


bool sendSMSLowLevel(String message) {
    String sendToString = numberMessage;
    
    char sendto[15] = numberMessage;
    char buffMessage[145];

    message.toCharArray(buffMessage, 145);

    if (!fona.sendSMS(sendto, buffMessage)) {
        #ifdef DEBUG_MODE
        Serial.print("Sending message to " + sendToString + " <---\n");
        #endif
    }else{
        #ifdef DEBUG_MODE
        Serial.print("Failed sending message <---\n");
        #endif
    }

}

void refreshConsumption(bool param1) {
    //param1 describe the kind of function aproximate 
    if (param1) {
        currentConsumption = currentVolume/speedKmh; // For Repair

        if (currentConsumption>consumptionThreshold) {
            alertState = 1;
        }
        // Complete this part
    }else{
        
        //alertState = int(func(currentConsumption));

    }
   


}


void sendAlertToBoss() {
    String finalMessage = "";
    finalMessage += "Angles:" + String(Angle[0]) + "," + String(Angle[1]) + ";";
    finalMessage += "Lat:" + String(currentLatitude);
    finalMessage += "Lon:" + String(currentLongitude) + "\n";
    finalMessage += "Threshold activated, Check it the datalog\n";
    finalMessage += "MARC project";


    #ifdef DEBUG_MODE
    Serial.println("Final Alert message complied:");
    Serial.println(finalMessage);
    #endif
    
    sendSMSLowLevel(finalMessage);


}




