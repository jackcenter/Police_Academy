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

int leftSpeed;              // PWM 0-255
int rightSpeed;             // PWM 0-255
long leftPos;               // Encoder value
long rightPos;              // Encoder value

Encoder encLeft(ENC1A, ENC1B);
Encoder encRight(ENC2A, ENC2B);

void setup() {
  leftSpeed = 50;
  leftPos = 0;
  
  rightSpeed = 50;
  rightPos = 0;
  
  // Start I2C Bus as Slave
  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(9600);
  Serial.println("Encoder Test:");
}

void loop() {
  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);
  readEncoders();
  delay(100);
}

void receiveEvent(int howMany) 
{
  int numOfBytes = Wire.available();

  Serial.print("len:");
  Serial.println(numOfBytes);
  Wire.read();            // throws away first byte
  
  char actionLeft = (int)Wire.read();
  char actionRight = (int)Wire.read();
  Serial.print(actionLeft + ", " + actionRight);

  accelerateMotor(leftSpeed, actionLeft);
  accelerateMotor(rightSpeed, actionRight);
}  

void serialEvent()
{
 // while (Serial.available()) {
  int numOfBytes = Serial.available();
  Serial.print("len:");
  Serial.println(numOfBytes);
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
  //}
}

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
  Serial.print("Left encoder: ");
  Serial.println(leftPos);
  Serial.print("Right encoder: ");
  Serial.println(rightPos);
}
