#include <Encoder.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Range.h>


//2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13
#define AIN1 9
#define AIN2 10
#define PWMA 11
#define ENC1A 2
#define ENC1B 12
#define BIN1 7
#define BIN2 6
#define PWMB 5
#define ENC2A 3
#define ENC2B 13
#define STBY 8

byte slave_address = 7;

const double radius = 2;    // radius of the wheel in inches
const double axel = 10;     // distance between wheels
const int cpr = 8400;       // ecoder spec
const int res = 64;         // number of encoder counts per single turn

const int offsetLeft = -1;
const int offsetRight = 1;
Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetLeft, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetRight, STBY);

int leftSpeed;              // PWM 0-255
int rightSpeed;             // PWM 0-255
long leftPos;               // Encoder value
long rightPos;              // Encoder value

double frontD=0.0;
double rightD=0.0;
double leftD=0.0;

ros::NodeHandle nh;

void cb_front(const sensor_msgs::Range& range_front) {
  frontD = range_front.range;
}

void cb_right(const sensor_msgs::Range& range_right) {
  rightD = range_right.range;
}

void cb_left(const sensor_msgs::Range& range_left) {
  leftD = range_left.range; 
}

ros::Subscriber<sensor_msgs::Range> sub_front("ultrasonic_front", &cb_front);
ros::Subscriber<sensor_msgs::Range> sub_right("ultrasonic_right", &cb_right);
ros::Subscriber<sensor_msgs::Range> sub_left("ultrasonic_left", &cb_left);

Encoder encLeft(ENC1A, ENC1B);
Encoder encRight(ENC2A, ENC2B);

void setup() {
  leftSpeed = 150;
  leftPos = 0;
  
  rightSpeed = 150;
  rightPos = 0;

  motorLeft.drive(50);
  motorRight.drive(50);
  
  nh.initNode();
  
  
  // Start I2C Bus as Slave
  Wire.begin(slave_address);
//  Wire.onReceive(receiveEvent);
  
//  Serial.begin(9600);
//  Serial.println("Encoder Test:");
}

void loop() {

  nh.subscribe(sub_front);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);

  if(  ((leftD)>0.0 && (leftD)<8.0) && ((rightD)>0.0 && (rightD)<7.0) )  
  {
    motorLeft.drive(200);
    motorRight.drive(200);
  }
  
  else if( leftD>8.0 && leftD<15.0)
  {
    if(leftSpeed > 0 && leftSpeed < 230) {
    if(rightSpeed > 0 && rightSpeed < 230){
//      motorLeft.drive(leftSpeed-50);
//      motorRight.drive(rightSpeed+50);
      motorLeft.brake();
      motorRight.drive(200);
      }
  
    }
  }
  else if(rightD>8.0  && rightD<15.0)
  {
    if(leftSpeed > 0 && leftSpeed < 230) {
    if(rightSpeed > 0 && rightSpeed < 230){
//      motorLeft.drive(rightSpeed-50);
//      motorRight.drive(leftSpeed+50);
        motorLeft.drive(200);
      motorRight.brake();
      }
  
    }
  }
  else 
  {
    motorLeft.drive(0);
    motorRight.drive(0);
  }

  nh.spinOnce();
//  readEncoders();/
  delay(100);
}

void receiveEvent(int howMany) 
{
  int numOfBytes = Wire.available();
  Wire.read();            // throws away first byte
  
  char actionLeft = (int)Wire.read();
  char actionRight = (int)Wire.read();

  accelerateMotor(leftSpeed, actionLeft);
  accelerateMotor(rightSpeed, actionRight);
}  

<<<<<<< HEAD
//void serialEvent()
//{
// // while (Serial.available()) {
//  int numOfBytes = Serial.available();
////  Serial.print("len:");
////  Serial.println(numOfBytes);
////  char inputLeft = (char)Serial.read();
////  char inputRight = (char)Serial.read();
////  Serial.read();
//
//  int actionLeft = convertInput(inputLeft, 5);
//  int actionRight = convertInput(inputRight, 5);
//
////  Serial.println(actionLeft);
////  Serial.println(actionRight);
//
//  accelerateMotor(leftSpeed, actionLeft);
//  accelerateMotor(rightSpeed, actionRight);
//
////  Serial.println(leftSpeed);
////  Serial.println(rightSpeed);
//  //}
//}
=======
void serialEvent()
{
 // while (Serial.available()) {
  int numOfBytes = Serial.available();
  Serial.print("len:");
  Serial.println(numOfBytes);

  if (numOfBytes == 3){
  { 
    char inputLeft = (char)Serial.read();
    char inputRight = (char)Serial.read();
    Serial.read();
  
    int actionLeft = convertInput(inputLeft, 5);
    int actionRight = convertInput(inputRight, 5);
  
    Serial.println(actionLeft);
    Serial.println(actionRight);
  
    accelerateMotor(leftSpeed, actionLeft);
    accelerateMotor(rightSpeed, actionRight);
  
    Serial.println(leftSpeed);
    Serial.println(rightSpeed);
  }

  else if (numOfBytes == 4){
    Serial.read();
    char inputLeft = (char)Serial.read();
    char inputRight = (char)Serial.read();
    Serial.read();

    accelerateMotorToValue
  }
}
>>>>>>> 0a51eb50b55dda46b0bb4eea8ede80eb6308b96d

int convertInput(char input, int multiple)
{
  int conversion = 0;
  
  if (input == 'q'){
    conversion = multiple;
  }

  else if (input == 'a'){
    conversion = 0;
  }

  else if (input == 'z'){
    conversion = -multiple;
  }

  return conversion;
}

void accelerateMotor(int &currentSpeed, int accel)
{
  // currentSpeed is the current PWM setting
  // accel is the PWM increment
  // TODO: need to add error checking if commanded beyond 0-255
  currentSpeed += accel;
}

void readEncoders()
{
  leftPos = offsetLeft*encLeft.read()/(cpr/res);
  rightPos = offsetRight*encRight.read()/(cpr/res);
}
