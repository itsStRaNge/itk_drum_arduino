#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include <MadgwickAHRS.h>

Madgwick filter;

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
          Serial.print(filter.getRoll());
          Serial.print(",");
          Serial.print(filter.getPitch());
          Serial.print(",");
          Serial.println(filter.getYaw());

          /*
          Serial.println(myIMU.readFloatAccelX());
          Serial.println(myIMU.readFloatAccelY());
          Serial.println(myIMU.readFloatAccelZ());
          Serial.println(myIMU.readFloatGyroX());
          Serial.println(myIMU.readFloatGyroY());
          Serial.println(myIMU.readFloatGyroZ());
          */
    }
