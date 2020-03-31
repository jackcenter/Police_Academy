#include <Encoder.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>

union Buffer
{
    long longNumber;
    byte longBytes[4];
};

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
Buffer buffer;

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
  rightSpeed = 0;
  rightPos = 0;
  
  leftSpeed = 0;
  leftPos = 0;

  // Start I2C Bus as Slave
  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendEvent);
  
  Serial.begin(9600);
  Serial.println("Encoder Test:");
}

void loop() {
  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);
  readEncoders();
}

void receiveEvent(int howMany) 
{
  int numOfBytes = Wire.available();

  if (numOfBytes != 3){
    return;
  }
  
  Serial.print("Command, len: ");
  Serial.println(numOfBytes);
  Wire.read();            // throws away first byte

  if (numOfBytes == 3){
    char u1_in = (int)Wire.read();
    char u2_in = (int)Wire.read();
  
    // get integers from wire command
//    int u1 = convertInput(u1_in);
//    int u2 = convertInput(u2_in);
    int u1 = adjustInput(u1_in);
    int u2 = adjustInput(u2_in);
  
    Serial.print(" u1: ");
    Serial.println(u1);
    Serial.print(" u2: ");
    Serial.println(u2);
    Serial.println();
  
    // convert input into acceleration for each motor [right, left]
    int omega_dot1[] = {u1, u1};
    int rem = u2%2;
    int omega_dot2[] = {u2/2 + rem, -u2/2};
    int* omega_dot = add_arrays(omega_dot1, omega_dot2);
  
    accelerateMotor(rightSpeed, omega_dot[0]);
    accelerateMotor(leftSpeed, omega_dot[1]);
  
    Serial.print(" Right speed:    ");
    Serial.println(rightSpeed);
    Serial.print(" Left speed:     ");
    Serial.println(leftSpeed);
  }  
}

void sendEvent()
{
  int numOfBytes = Wire.available();
  Serial.print("len: ");
  Serial.println(numOfBytes);
  byte side = Wire.read();
  Serial.print("Send: ");
  Serial.println(side);
  if (side == 0){
      buffer.longNumber = rightPos;     
      Wire.write(buffer.longBytes, 4);
      buffer.longNumber = leftPos;
      Wire.write(buffer.longBytes, 4);
  }

  else if (side == 1){
      buffer.longNumber = leftPos;
      Wire.write(buffer.longBytes, 4);
  }


  Serial.print(" Right position: ");
  Serial.println(rightPos);
  Serial.print(" Left position:  ");
  Serial.println(leftPos);
  Serial.println();
}
  

int convertInput(char input)
{
  int value = input - '0';
  value += -3;
  return value;
}

int adjustInput(int value)
{
  value += -3;
  return value;
}

String convertIntToStr(int val)
{
  String conversion = String(val);
  return conversion;
}

int* add_arrays(int a[], int b[])
{
  static int c[2];
  int i;
  
  for (i = 0; i < 2; ++i){
    c[i] = a[i] + b[i];
  }

  return c;
}

void accelerateMotor(int &currentSpeed, int accel)
{
  // currentSpeed is the current PWM setting
  // accel is the PWM increment
  // TODO: need to add error checking if commanded beyond 0-255
  currentSpeed += accel;

  if (currentSpeed > 255){
    currentSpeed = 255;
  }

  else if (currentSpeed < 0){
    currentSpeed = 0;
  }
}

void readEncoders()
{
  leftPos = offsetLeft*encLeft.read()/(cpr/res);
  rightPos = offsetRight*encRight.read()/(cpr/res);
//  Serial.print("Left encoder: ");
//  Serial.println(leftPos);
//  Serial.print("Right encoder: ");
//  Serial.println(rightPos);
}
