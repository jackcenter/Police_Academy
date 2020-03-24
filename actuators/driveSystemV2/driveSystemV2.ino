#include <Encoder.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>

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
int leftBaseSpeed = 255;
int rightBaseSpeed = 225;


Encoder encLeft(ENC1A, ENC1B);
Encoder encRight(ENC2A, ENC2B);

void setup() {
  // Start I2C Bus as Slave
  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(9600);
  Serial.println("Encoder Test:");
}

//long leftPos = 0;
//long rightPos = 0;

void loop() {
//  forward(30);
//  delay(1000);
//  backward(30);
//  delay(1000);
//  right(90);
//  delay(1000);
//  left(90);
//  delay(1000);
}

void receiveEvent(int howMany) 
{
  int numOfBytes = Wire.available();
  int val = 10;

  Serial.print("len:");
  Serial.println(numOfBytes);
  // needs to receive a character command for direction then an integer command for distance/angle
  // char cmd = char(Wire.read());

  // Need to read a value and a distance
//  int val = Wire.read();
  Wire.read();
  char action = Wire.read();
  // Serial.print(cmd);
  Serial.print(action);

//  for(int i=1; i<numOfBytes-1; i++){
//    
//    action = Wire.read();
//    Serial.print(data);
//  }

  
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
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = offsetLeft*encLeft.read()/(cpr/res);
    rightPos = offsetRight*encRight.read()/(cpr/res);
    Serial.print("Left encoder: ");
    Serial.println(leftPos);
    Serial.print("Right encoder: ");
    Serial.println(rightPos);
     
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
    Serial.print("Left encoder: ");
    Serial.println(leftPos);
    Serial.print("Right encoder: ");
    Serial.println(rightPos);
    
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
