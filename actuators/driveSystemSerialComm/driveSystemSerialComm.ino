#include <Encoder.h>
#include <SparkFun_TB6612.h>

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

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete

const double radius = 2;    // radius of the wheel in inches
const double axel = 10;     // distance between wheels
const int cpr = 8400;       // ecoder spec
const int res = 84;         // number of encoder counts per single turn

const int offsetLeft = -1;
const int offsetRight = 1;
Motor motorRight = Motor(AIN1, AIN2, PWMA, offsetLeft, STBY);
Motor motorLeft = Motor(BIN1, BIN2, PWMB, offsetRight, STBY);

int leftSpeed;              // PWM 0-255
int rightSpeed;             // PWM 0-255
long leftPos;               // Encoder value
long rightPos;              // Encoder value

Encoder encRight(ENC1A, ENC1B);
Encoder encLeft(ENC2A, ENC2B);

void setup() {
  rightSpeed = 0;
  rightPos = 0;
  encRight.write(rightPos*(cpr/res));
  
  leftSpeed = 0;
  leftPos = 0;
  encLeft.write(leftPos*(cpr/res));
  
  Serial.begin(9600);
}

void loop() {
//  while (Serial.available() > 0){
//    processIncomingByte(Serial.read());
//  }
  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);
  readEncoders();
  Serial.print(leftPos);
  delay(10);
}

void processIncomingByte(const byte inByte)
{
  char inChar = (char)inByte;
  switch (inByte)
  {
    case '\n':
      processData(inputString);
      inputString = "";
      break;
      
    default:
      inputString += inChar;
      break;
  }
}

void processData(String data)
{
  if (inputString == "l"){
    Serial.println(leftPos);
  }
    
  else if (inputString == "r"){    
    Serial.println(rightPos);
  }

  else {
    Serial.println("Oops");
  }
  
}

//void serialEvent(){
//  while (Serial.available()) {
//    char inChar = (char)Serial.read();
//    inputString += inChar;
//
//    if (inChar == '\n') {
//      stringComplete = true;
//    }
//  }
//
//  if (stringComplete == true){
//    readEncoders();
//
//    if (inputString == "l\n"){
//      Serial.println(leftPos);
//    }
//    
//    else if (inputString == "r\n"){    
//      Serial.println(rightPos);
//    }
//
//    inputString = "";
//    stringComplete = false;
//  }
//}

int convertInput(char input)
{
  int value = input - '0';
  value += -6;
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

  else if (currentSpeed < -255){
    currentSpeed = -255;
  }
}

void readEncoders()
{
  leftPos = encLeft.read()/(cpr/res);
  rightPos = -encRight.read()/(cpr/res);
//  Serial.print("Left encoder: ");
//  Serial.println(leftPos);
//  Serial.print("Right encoder: ");
//  Serial.println(rightPos);
}
