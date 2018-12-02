#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include <MadgwickAHRS.h>

#include <SoftwareSerial.h>   //Software Serial Port
//#include <ArduinoJson.h>
#define RxD 0
#define TxD 1
#define DEBUG_ENABLED  1
SoftwareSerial BLE(RxD,TxD);
//DynamicJsonBuffer jBuffer;
//JsonObject &root= jBuffer.createObject();
float sensor_buffer;
Madgwick filter;

//2.8 
 
//INT HAS TWO BYTES
struct message{
  unsigned int counter;
  int Acc_X;
  int Acc_Y;
  int Acc_Z;
  int Gyro_X;
  int Gyro_Y;
  int Gyro_Z;
  int roll;
  int pitch;
  int yaw;
};

struct message msg;
char msg_string[sizeof(struct message)];

LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A


    void setup()
    {
      Serial.begin(9600);
      filter.begin(50);

      msg.counter=0;
      msg.Acc_X=0;
      msg.Acc_Y=0;
      msg.Acc_Z=0;
      msg.Gyro_X=0;
      msg.Gyro_Y=0;
      msg.Acc_Z=0;
      
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
        msg.counter = msg.counter+1;
        msg.Acc_X = (int) (myIMU.readFloatAccelX()*100);
        msg.Acc_Y = (int)(myIMU.readFloatAccelY()*100);
        msg.Acc_Z = (int) (myIMU.readFloatAccelZ()*100);
        msg.Gyro_X =  (int) (myIMU.readFloatGyroX()*100);
        msg.Gyro_Y =  (int) (myIMU.readFloatGyroY()*100);
        msg.Gyro_Z =  (int) (myIMU.readFloatGyroZ()*100);
        msg.roll = (int) filter.getRoll()*100;
        msg.pitch= (int) filter.getPitch()*100;
        msg.yaw = (int)  filter.getYaw()*100;                  

        
        /*
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
          */
          Serial.write((byte*)&msg, sizeof(struct message));

          //memcpy(msg_string, &msg, sizeof(struct message));
          //Serial.print(msg_string);
    }

    void setupBleConnection()
    {
      BLE.begin(9600); //Set BLE BaudRate to default baud rate 9600
      BLE.print("AT+CLEAR"); //clear all previous setting
      BLE.print("AT+ROLE0"); //set the bluetooth name as a slaver
      BLE.print("AT+SAVE1");  //don't save the connect information
      Serial.print("begin transmission");
      Serial.println("");
    }

