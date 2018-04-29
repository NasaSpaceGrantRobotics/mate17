
#include <Servo.h> 
#include <SoftwareSerial.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>  //where the bluetooth information is going to 
#include <std_msgs/Float32.h>

#define FRONT_RIGHT_PIN    3
#define FRONT_LEFT_PIN     5
#define BACK_RIGHT_PIN     8
#define BACK_LEFT_PIN      7
#define FRONT_VERT_PIN     10
#define BACK_VERT_PIN      9
#define CLAW_GRAB_PIN      45
#define CLAW_ROTATION_PIN  46
#define LIGHT_PIN          44

#define RX_PIN 19
#define TX_PIN 18

//init ros node need to run ros
ros::NodeHandle  nh;

std_msgs::Int16MultiArray motors_back;
ros::Publisher pub_back("motor_back", &motors_back);

std_msgs::String str_msg; //creating a string of std_msgs to publish to 
ros::Publisher bluetooth("bluetooth", &str_msg);  //creeating a topic to access the data

SoftwareSerial SlaveSerial(RX_PIN, TX_PIN);    // Rx and Tx Pins respectively on the Arduino Mega. 
                                               // Should be connected such that Rx --> Tx of Slave HC-06 chip AND 
                                               // Tx --> Rx of Slave HC-06 chip

std_msgs::Float32 mag;
std_msgs::Int16MultiArray accel;
std_msgs::Int16MultiArray gyro;

ros::Publisher pub_mag("magnetometer", &mag);
ros::Publisher pub_accel("accelerometer", &accel);
ros::Publisher pub_gyro("gyroscope", &gyro);

char tempChar;
int index = 0;
char crateCode[12];

MPU6050 accelgyro; // address = 0x68, the default
int16_t  gx, gy, gz;
int16_t ax, ay, az;
int16_t mx, my, mz;

int16_t accel_out[3];
int16_t gyro_out[3];

float heading;

int16_t back[8];

//creating the servos for the motors
Servo front_right;
Servo front_left;

Servo back_right;
Servo back_left;

Servo front_vertical;
Servo back_vertical;

Servo claw_rotation;
Servo claw_grab;

Servo light;
/*

0   +   1
/-------\
|   4   |
|       |
|   5   |
\-------/
2   +   3

data value array locations for the motors


*/


//light true = 1900, ligth false = 1100;

void motor_value_pull( const std_msgs::Int16MultiArray& values){
  
  front_right.writeMicroseconds(values.data[0]);
  front_left.writeMicroseconds(values.data[1]);
  
  back_right.writeMicroseconds(values.data[2]);
  back_left.writeMicroseconds(values.data[3]);

  front_vertical.writeMicroseconds(values.data[4]);
  back_vertical.writeMicroseconds(values.data[5]);

  claw_grab.writeMicroseconds(values.data[6]);
  claw_rotation.writeMicroseconds(values.data[7]);

  back[0] = values.data[0];
  back[1] = values.data[1];
  back[2] = values.data[2];
  back[3] = values.data[3];
  back[4] = values.data[4];
  back[5] = values.data[5];
  back[6] = values.data[6];
  back[7] = values.data[7];
   
}

void light_pull(const std_msgs::Bool& value){

  if(value.data) light.writeMicroseconds(1900);
  else light.writeMicroseconds(1100);
  
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_motor("motorValues", motor_value_pull);
ros::Subscriber<std_msgs::Bool> sub_light("light", light_pull);

void setup(){

  nh.initNode();
  
  nh.subscribe(sub_motor);
  nh.subscribe(sub_light);
  
  motors_back.data_length = 8;
  
  nh.advertise(pub_back);
  nh.advertise(bluetooth);

  gyro.data_length = 3;
  accel.data_length = 3;
    
  nh.advertise(pub_mag);
  nh.advertise(pub_accel);
  nh.advertise(pub_gyro);
 
    
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(57600);
  
  front_right.attach(FRONT_RIGHT_PIN);
  front_left.attach(FRONT_LEFT_PIN);
  
  back_right.attach(BACK_RIGHT_PIN);
  back_left.attach(BACK_LEFT_PIN);

  front_vertical.attach(FRONT_VERT_PIN);
  back_vertical.attach(BACK_VERT_PIN);

  claw_grab.attach(CLAW_GRAB_PIN);
  claw_rotation.attach(CLAW_ROTATION_PIN);

  light.attach(LIGHT_PIN);

  delay(2000);

  Serial.begin(57600);
  SlaveSerial.begin(9600);       // Set the baud rate of the Slave bluetooth module to 9600 bits per second 
                                 // so that it "understands" the transmission speed of the Master bluetooth module 
                                 // (which is also set to a baud rate of 9600)
  delay(2000);
  
}

void loop(){
  

  if (SlaveSerial.available() > 0){        // Do NOT try to execute this code until enough characters are being received by the Slave bluetooth module
  
    tempChar = SlaveSerial.read();        // Each byte (single char) of the transferred "code" to the Slave bluetooth module is analyzed during a single loop iteration after being assigned to 'tempChar'
    
    if (tempChar == '\r'){                 // Checks for the "code's" end sequence of a "carriage return" ---> '\r'
        
        if (index >= 7){                   // index is no longer in range of the crateCode array
          
          crateCode[index] = 0;           // ASCII value of null
          
          for (index = 0; crateCode[index] != '\0'; index++){
            Serial.write(crateCode[index]);
          }
          
          Serial.println();             // Returns after every full crate code is displayed on the Serial Monitor. Its purpose is for readibility.
        }
        
        index = 0;                      // Reset the index to the start position 
    
    }
    
    else{
      if (index >= 11){                  // Leave space for the null termination. Can now hold longer strings.
      
        index = 0;                      // Keep writing like normal
      
      }
      
      crateCode[index] = tempChar;
      
      index++;
    }

  }

  motors_back.data = back;
  pub_back.publish(&motors_back);
  
  str_msg.data = crateCode;
  bluetooth.publish(&str_msg);

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro.getMag(&mx, &my, &mz);

  heading = atan2((double)my, (double)mx) * 180.0/3.14159265 + 180;
  while (heading < 0) heading += 360;
  while (heading > 360) heading -= 360;
 
   
  accel_out[0] = ax;
  accel_out[1] = ay;
  accel_out[2] = az;

  gyro_out[0] = gx; 
  gyro_out[1] = gy;
  gyro_out[2] = gz;

  mag.data = heading; 
  accel.data = accel_out; 
  gyro.data = gyro_out; 
    
    
  pub_mag.publish(&mag);
  pub_accel.publish(&accel);
  pub_gyro.publish(&gyro);
  
  nh.spinOnce();

  delay(10);
}
