#include <Encoder.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


//ROS STUFF
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x,float out_min, float out_max);
#define PWM_MIN 50
#define PWM_RANGE 200
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&onTwist);



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


//void cb_front(const sensor_msgs::Range& range_front) {
//  frontD = range_front.range;
//}
//
//void cb_right(const sensor_msgs::Range& range_right) {
//  rightD = range_right.range;
//}
//
//void cb_left(const sensor_msgs::Range& range_left) {
//  leftD = range_left.range; 
//}
//
//ros::Subscriber<sensor_msgs::Range> sub_front("ultrasonic_front", &cb_front);
//ros::Subscriber<sensor_msgs::Range> sub_right("ultrasonic_right", &cb_right);
//ros::Subscriber<sensor_msgs::Range> sub_left("ultrasonic_left", &cb_left);

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
  nh.subscribe(sub);  // Cmd_vel drive based on velocity
//  nh.subscribe(sub_front); // Subscribing Front Ultrasonic Range
//  nh.subscribe(sub_right); // Subscribing Left Ultrasonic Range
//  nh.subscribe(sub_left);  // Subscribing Right Ultrasonic Range
  // Start I2C Bus as Slave
  Wire.begin(slave_address);
}


void onTwist(const geometry_msgs::Twist& msg)
{
  //Capping values [-1 ... 1]
  float x= max( min(msg.linear.x,1.0f), -1.0f);
  float z= max( min(msg.angular.z, 1.0f), -1.0f);

  //Calculating the intensity of left and right wheels 
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  //Mapping those values to PWM intensities, PWMRANGE = fullspeed, PWMMIN = 50
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWM_RANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWM_RANGE);

  motorLeft.drive(lPwm);
  motorRight.drive(rPwm);
}


void loop() {

  nh.spinOnce();
}

// Mapping x value from [0 ... 1] to [out_min .. outmax]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

//void receiveEvent(int howMany) 
//{
//  int numOfBytes = Wire.available();
//  Wire.read();            // throws away first byte
//  
//  char actionLeft = (int)Wire.read();
//  char actionRight = (int)Wire.read();
//
//  accelerateMotor(leftSpeed, actionLeft);
//  accelerateMotor(rightSpeed, actionRight);
//}  

void readEncoders()
{
  leftPos = offsetLeft*encLeft.read()/(cpr/res);
  rightPos = offsetRight*encRight.read()/(cpr/res);
}
