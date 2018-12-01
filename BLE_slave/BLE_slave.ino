#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include <MadgwickAHRS.h>

#include <SoftwareSerial.h>   //Software Serial Port
#include <ArduinoJson.h>
#define RxD 0
#define TxD 1
#define DEBUG_ENABLED  1
SoftwareSerial BLE(RxD,TxD);
DynamicJsonBuffer jBuffer;
JsonObject &root= jBuffer.createObject();
float sensor_buffer;
Madgwick filter;
int counter = 0;

LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A
    void setup()
    {
      Serial.begin(9600);
      filter.begin(50);


      if( myIMU.begin() != 0 )
      {
        Serial.println("Device error");
      }
      else  
      {
          Serial.println("Device OK!");
      }
      pinMode(RxD, INPUT);
      pinMode(TxD, OUTPUT);
      setupBleConnection();
    }
    
    void loop()
    {
        filter.updateIMU(myIMU.readFloatGyroX(),
                         myIMU.readFloatGyroY(),
                         myIMU.readFloatGyroZ(),
                         myIMU.readFloatAccelX(),
                         myIMU.readFloatAccelY(),
                         myIMU.readFloatAccelZ());
       
        delay(20);
          root["roll"]=filter.getRoll(); 
          root["pitch"]=filter.getPitch();
          root["yaw"]= filter.getYaw();
          sensor_buffer = myIMU.readFloatAccelX();
          root["Acc_X"]=sensor_buffer;
          sensor_buffer = myIMU.readFloatAccelY();
          root["Acc_Y"]=sensor_buffer;
          sensor_buffer = myIMU.readFloatAccelZ();
          root["Acc_Z"]=sensor_buffer;
          sensor_buffer = myIMU.readFloatGyroX();
          root["Gyro_X"]=sensor_buffer;
          sensor_buffer = myIMU.readFloatGyroY();
          root["Gyro_Y"]=sensor_buffer;
          sensor_buffer = myIMU.readFloatGyroZ();
          root["Gyro_Z"]=sensor_buffer;
          root.prettyPrintTo(Serial);
          Serial.println();
    }

    void setupBleConnection()
    {
      BLE.begin(9600); //Set BLE BaudRate to default baud rate 9600
      BLE.print("AT+CLEAR"); //clear all previous setting
      BLE.print("AT+ROLE0"); //set the bluetooth name as a slaver
      BLE.print("AT+SAVE1");  //don't save the connect information
      Serial.print("begin transmission");
    }
