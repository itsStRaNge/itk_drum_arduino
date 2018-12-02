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
Madgwick filter;

float value_buffer;

float ma_acc=1;
float last_acc=0;

float ma_down_vel=1;
float eps_1 = 0.2;
float eps_2 = 0.08;
int lock=0;


struct message{
  char drum_id;
  char strength;
};

struct message msg;

LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

    void setup()
    {
      Serial.begin(9600);
      filter.begin(100);
      msg.drum_id = 1;
      
      if(myIMU.begin() != 0 )
      {
        
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
       
        delay(10);
        if(lock>0){
          lock--;
        }

        
        float roll = filter.getRoll();
        float pitch = filter.getPitch();
        float yaw = filter.getYaw();
        
        value_buffer=0;
        value_buffer +=myIMU.readFloatAccelX()*myIMU.readFloatAccelX();
        value_buffer +=myIMU.readFloatAccelY()*myIMU.readFloatAccelY();
        value_buffer +=myIMU.readFloatAccelZ()*myIMU.readFloatAccelZ();
        value_buffer = sqrt(value_buffer);
        
        last_acc = ma_acc;
        ma_acc = eps_1*value_buffer + (1-eps_1)*ma_acc;
        ma_down_vel = eps_2*myIMU.readFloatGyroX() + (1-eps_2)*ma_down_vel;

        //max strength is 15
        
  
        if(ma_acc>4.0 && ma_acc < last_acc && ma_down_vel > 40 && lock==0){
          if(roll<-25){
            //upper drum hit
            msg.drum_id=1;
            msg.strength=last_acc*7;
          }
          else{
          //lower drum hit
            msg.drum_id=2;
            msg.strength=last_acc*7;
          }
           if(msg.strength>100){
              msg.strength=100;
            }
            Serial.write((byte*)&msg, sizeof(struct message));
            digitalWrite(13,HIGH);
            delay(50);
            digitalWrite(13,LOW);
            
          lock=10;
        }
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

