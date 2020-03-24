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

int leftSpeed = 0;
int rightSpeed = 0;

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
  // inputs
  // if a change in speeds is sensed:
  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  encPosLeft = encLeft.read();
  encPosRight = encRight.read();
}

void receiveEvent(int howMany) 
{
  char action = Wire.read();

  // Need to be able to receive the input vector or send the encoder information.
  switch (action){
      case 'f':
        Serial.println("Received command")
        break;
      case 's':
        Serial.println("Send data")
        Wire.write(
        break;
      case 'l':
        left(val);
        break;
      case 'r':
        right(val);
        break;
      case
    }
}
