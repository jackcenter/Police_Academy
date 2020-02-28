#include <Encoder.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>

#define AIN1 12
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9
#define ENC1A 2
#define ENC1B 10 
#define ENC2A 3
#define ENC2B 13

byte slave_address = 7;

const double radius = 2;    // radius of the wheel in inches
const double axel = 10;     // distance between wheels
const int cpr = 8400;       // ecoder spec
const int res = 16;         // number of encoder counts per single turn

const int offsetLeft = -1;
const int offsetRight = 1;
Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetLeft, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetRight, STBY);


Encoder encLeft(ENC1A, ENC1B);
Encoder encRight(ENC2A, ENC2B);

void setup() {
  // Start I2C Bus as Slave
  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(9600);
  Serial.println("Encoder Test:");

}

void loop() {
  // put your main code here, to run repeatedly:

}

void receiveEvent(int howMany) 
{
  // needs to receive a character command for direction then an integer command for distance/angle
  char cmd = Wire.read();
  // Need to read a value and a distance
  int val = Wire.read();

  switch (cmd){
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
  int leftPos = 0;
  int rightPos = 0;
  int leftBaseSpeed = 255;
  int rightBaseSpeed = 255;
  
  int steps = x/(2*PI*radius)*res; 
  int leftGoal = steps;
  int rightGoal = steps;
  int leftSpeed = leftBaseSpeed;
  int rightSpeed = rightBaseSpeed;

  while (leftPos < leftGoal || rightPos < rightGoal)
  {
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = abs(offsetLeft*encLeft.read()/(cpr/res));
    rightPos = abs(offsetRight*encRight.read()/(cpr/res));
    
    rpmControl(leftSpeed, rightSpeed, leftPos, rightPos, leftBaseSpeed, rightBaseSpeed);
    
    if (leftPos >= leftGoal){
        motorLeft.brake();
    }
    
    if (rightPos >= rightGoal){
        motorRight.brake();
    }   
     
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }
}

void backward(int x)
{
  int leftPos = 0;
  int rightPos = 0;
  int leftBaseSpeed = -255;
  int rightBaseSpeed = -255;
  
  int steps = x/(2*PI*radius)*res; 
  int leftGoal = steps;
  int rightGoal = steps;
  int leftSpeed = leftBaseSpeed;
  int rightSpeed = rightBaseSpeed;

  while (leftPos < leftGoal || rightPos < rightGoal)
  {
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = abs(offsetLeft*encLeft.read()/(cpr/res));
    rightPos = abs(offsetRight*encRight.read()/(cpr/res));
    
    rpmControl(leftSpeed, rightSpeed, leftPos, rightPos, leftBaseSpeed, rightBaseSpeed);
    
    if (leftPos >= leftGoal){
        motorLeft.brake();
    }
    
    if (rightPos >= rightGoal){
        motorRight.brake();
    }   
     
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }
}

void left(int deg)
{
  int leftPos = 0;
  int rightPos = 0;
  int leftBaseSpeed = 255;
  int rightBaseSpeed = -255;

  int steps = axel/radius/720*deg*res; 
  int leftGoal = steps;
  int rightGoal = steps;
  int leftSpeed = leftBaseSpeed;
  int rightSpeed = rightBaseSpeed;

  while (leftPos < leftGoal || rightPos < rightGoal)
  {
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = abs(offsetLeft*encLeft.read()/(cpr/res));
    rightPos = abs(offsetRight*encRight.read()/(cpr/res));
    
    rpmControl(leftSpeed, rightSpeed, leftPos, rightPos, leftBaseSpeed, rightBaseSpeed);
    
    if (leftPos >= leftGoal){
        motorLeft.brake();
    }
    
    if (rightPos >= rightGoal){
        motorRight.brake();
    }   
     
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }
}

void right(int deg)
{
  int leftPos = 0;
  int rightPos = 0;
  int leftBaseSpeed = -255;
  int rightBaseSpeed = 255;

  int steps = axel/radius/720*deg*res; 
  int leftGoal = steps;
  int rightGoal = steps;
  int leftSpeed = leftBaseSpeed;
  int rightSpeed = rightBaseSpeed;

  while (leftPos < leftGoal || rightPos < rightGoal)
  {
    // TODO: Need some LED action here
    // TODO: Check if the RPM needs to be ramped up or down
    leftPos = abs(offsetLeft*encLeft.read()/(cpr/res));
    rightPos = abs(offsetRight*encRight.read()/(cpr/res));
    
    rpmControl(leftSpeed, rightSpeed, leftPos, rightPos, leftBaseSpeed, rightBaseSpeed);
    
    if (leftPos >= leftGoal){
        motorLeft.brake();
    }
    
    if (rightPos >= rightGoal){
        motorRight.brake();
    }   
     
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
  }
}

void rpmControl(int &leftSpeed, int &rightSpeed, int posLeft, int posRight, int leftBaseSpeed, int rightBaseSpeed)
{
  int pwmDrop = 10;
  int mag = posLeft - posRight;
  
  if (posLeft > posRight)
  {
    int signLeft = leftBaseSpeed / abs(leftBaseSpeed);
    leftSpeed = leftBaseSpeed - signLeft*mag*pwmDrop; 
    rightSpeed = rightBaseSpeed;
  }

  else if (posRight > posLeft)
  {
    int signRight = rightBaseSpeed / abs(rightBaseSpeed);
    leftSpeed = leftBaseSpeed;
    rightSpeed = rightSpeed - signRight*mag*pwmDrop; 
  }

  else 
  {
    leftSpeed = leftBaseSpeed;
    rightSpeed = rightBaseSpeed;
  }
}
