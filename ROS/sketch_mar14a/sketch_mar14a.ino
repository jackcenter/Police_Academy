#include<ArduinoHardware.h>
#include <SparkFun_TB6612.h>
#include<ros.h>
#include<geometry_msgs/Twist.h>
#include <Encoder.h>
#include <Wire.h>
#include<std_msgs/Int32.h>

//2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13
#define IN1_L 9
#define IN2_L 10
#define PWMA 11
#define ENC1A 2
#define EN_L 12

#define IN1_R 7
#define IN2_R 6
#define PWMB 5
#define ENC2A 3
#define EN_R 13

#define STBY 8

ros::NodeHandle nh;

//Publisher to test 
std_msgs::Int32 val_msg ;
ros::Publisher pub = nh.advertise<std_msgs::Int32>("chatter",1000);



int leftBaseSpeed = 150;
int rightBaseSpeed = 150;

Encoder encLeft(ENC1A, EN_L);
Encoder encRight(ENC2A, EN_R);
byte slave_address = 7;

double w_r=0, w_l=0;

double wheel_rad = 2,wheel_sep = 10;
const double radius = 2;    // radius of the wheel in inches
const double axel = 10;     // distance between wheels
const int cpr = 8400;       // ecoder spec
const int res = 64;         // number of encoder counts per single turn
const int offsetLeft = -1;
const int offsetRight = 1;


int lowSpeed = 50;
int HighSpeed = 200;
double speed_ang=0, speed_lin=0;
Motor motorLeft = Motor(IN1_L, IN2_L, PWMA, offsetLeft, STBY);
Motor motorRight = Motor(IN1_R, IN2_R, PWMB, offsetRight, STBY);

void cb(const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cb);
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup() {

   Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  
//  Serial.begin(9600);
  
  
  nh.initNode();
  nh.subscribe(sub);
  
}

void loop() {

//  std_msgs::Int32 val_msg;
//  val_msg = 3;
  pub.publish(val_msg);
  
  if( (w_l*10)>0 && (w_r*10)>0 ) forward(10);
  else if( (w_l*10)<0 && (w_r*10)<0) backward(10);
  else if( (w_l*10)>0 && (w_r*10)<0) left(10);
  else if((w_l*10)<0 && (w_r*10)>0) right(10);
  

  nh.spinOnce();
}


void Motors_init() {

  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  
//  Serial.begin(9600);
  
  Motor motorLeft = Motor(IN1_L, IN2_L, PWMA, offsetLeft, STBY);
  Motor motorRight = Motor(IN1_R, IN2_R, PWMB, offsetRight, STBY);
  
}











void receiveEvent(int howMany) 
{
  int numOfBytes = Wire.available();
  int val = 30;

//  Serial.print("len:");
//  Serial.println(numOfBytes);
  // needs to receive a character command for direction then an integer command for distance/angle
  char cmd = char(Wire.read());
  char action = Wire.read();

  
  switch (action){
      case 'f':
        forward(val);
        break;
      case 'b':
        backward(val);
        break;
      case 'l':
        left(val);
        break;
      case 'r':
        right(val);
        break;
    }
}

void forward(int x)
{
  int leftPos = offsetLeft*encLeft.read()/(cpr/res);
  int rightPos = offsetRight*encRight.read()/(cpr/res);
  int leftDir = 1;
  int rightDir = 1;
  
  int steps = x/(2*PI*radius)*res; 
  int leftGoal = leftPos + leftDir*steps;
  int rightGoal = rightPos + rightDir*steps;
  int leftSpeed = leftDir*leftBaseSpeed;
  int rightSpeed = rightDir*rightBaseSpeed;

  while (leftPos < leftGoal || rightPos < rightGoal)
  {

    leftPos = offsetLeft*encLeft.read()/(cpr/res);
    rightPos = offsetRight*encRight.read()/(cpr/res);
//    Serial.print("Left encoder: ");
//    Serial.println(leftPos);
//    Serial.print("Right encoder: ");
//    Serial.println(rightPos);
     
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }

  motorLeft.brake();
  motorRight.brake();
}

void backward(int x)
{
  int leftPos = offsetLeft*encLeft.read()/(cpr/res);
  int rightPos = offsetRight*encRight.read()/(cpr/res);
  int leftDir = -1;
  int rightDir = -1;
  
  int steps = x/(2*PI*radius)*res; 
  int leftGoal = leftPos + leftDir*steps;
  int rightGoal = rightPos + rightDir*steps;
  int leftSpeed = leftDir*leftBaseSpeed;
  int rightSpeed = rightDir*rightBaseSpeed;

  while (leftPos > leftGoal || rightPos > rightGoal)
  {

    leftPos = offsetLeft*encLeft.read()/(cpr/res);
    rightPos = offsetRight*encRight.read()/(cpr/res);
     
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }
  motorLeft.brake();
  motorRight.brake();
}

void left(int deg)
{
  int leftPos = offsetLeft*encLeft.read()/(cpr/res);
  int rightPos = offsetRight*encRight.read()/(cpr/res);
  int leftDir = -1;
  int rightDir = 1;

  int steps = axel/radius/720*deg*res; 
  int leftGoal = leftPos + leftDir*steps;
  int rightGoal = rightPos + rightDir*steps;
  int leftSpeed = leftDir*leftBaseSpeed;
  int rightSpeed = rightDir*rightBaseSpeed;

  while (leftPos > leftGoal || rightPos < rightGoal)
  {
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = offsetLeft*encLeft.read()/(cpr/res);
    rightPos = offsetRight*encRight.read()/(cpr/res);
        
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }

  motorLeft.brake();
  motorRight.brake();
}

void right(int deg)
{
  int leftPos = offsetLeft*encLeft.read()/(cpr/res);
  int rightPos = offsetRight*encRight.read()/(cpr/res);
  int leftDir = 1;
  int rightDir = -1;

  int steps = axel/radius/720*deg*res; 
  int leftGoal = leftPos + leftDir*steps;
  int rightGoal = rightPos + rightDir*steps;
  int leftSpeed = leftDir*leftBaseSpeed;
  int rightSpeed = rightDir*rightBaseSpeed;

  while (leftPos < leftGoal || rightPos > rightGoal)
  {
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = offsetLeft*encLeft.read()/(cpr/res);
    rightPos = offsetRight*encRight.read()/(cpr/res);
//    Serial.print("Left encoder: ");
//    Serial.println(leftPos);
//    Serial.print("Right encoder: ");
//    Serial.println(rightPos);
    
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }
  motorLeft.brake();
  motorRight.brake();
}

void rpmControl(int &leftSpeed, int &rightSpeed, int posLeft, int posRight, int leftBaseSpeed, int rightBaseSpeed)
{
  int pwmDrop = 5;
  int mag = posLeft - posRight;
  int signLeft = leftBaseSpeed / abs(leftBaseSpeed);
  int signRight = rightBaseSpeed / abs(rightBaseSpeed);
  
  if (posLeft > posRight)
  {
    leftSpeed = leftBaseSpeed - signLeft*mag*pwmDrop; 
    rightSpeed = rightBaseSpeed;
  }

  else if (posRight > posLeft)
  {
    leftSpeed = leftBaseSpeed;
    rightSpeed = rightSpeed - signRight*mag*pwmDrop; 
  }

  else 
  {
    leftSpeed = signLeft*leftBaseSpeed;
    rightSpeed = signRight*rightBaseSpeed;
  }
}
