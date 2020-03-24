#include <Encoder.h>
#include <SparkFun_TB6612.h>

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

const int offsetA = -1;
const int offsetB = 1;
const double radius = 2;    // radius of the wheel in inches
const double axel = 10;
const int cpr = 8400;
const int res = 10;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

Encoder encLeft(ENC1A, ENC1B);
Encoder encRight(ENC2A, ENC2B);

void setup()
{
 Serial.begin(9600);
 Serial.println("Encoder Test:");

}

long positionLeft = -999;
long positionRight = -999;
 
void loop()
{
  long newLeft = -encLeft.read()/(cpr/res);
//  if (newLeft != positionLeft)
//  {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
//  }

  long newRight = encRight.read()/(cpr/res);
//  if (newRight != positionRight)
//  {
    Serial.print("Right = ");
    Serial.print(newRight);
    Serial.println();
    positionRight = newRight;
//  }

  if (Serial.available())
  {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    encLeft.write(0);
  }
  Serial.println("Start cycle");
  forward(13);
  delay(1000);
  reverse(13);
  delay(1000);
  turnLeft(90);
  delay(1000);
  turnRight(90);
  delay(1000);
}

void forward(int dist)
{
  long rotations = dist/(2*PI*radius);
  long LeftFinalPos = positionLeft + rotations*res;
  long RightFinalPos = positionRight + rotations*res;

  motor1.drive(255);
  motor2.drive(255);
  while (positionLeft < LeftFinalPos || positionRight < RightFinalPos)
  {
    positionLeft = -encLeft.read()/(cpr/res);  
    positionRight = encRight.read()/(cpr/res); 

    if (positionLeft >= LeftFinalPos){
        motor1.brake();
    }

    if (positionRight >= RightFinalPos){
        motor2.brake();
    }       
  }
}

void reverse(int dist)
{
  long rotations = dist/(2*PI*radius);
  long LeftFinalPos = positionLeft - rotations*res;
  long RightFinalPos = positionRight - rotations*res;

  motor1.drive(-255);
  motor2.drive(-255);
  
  while (positionLeft > LeftFinalPos || positionRight > RightFinalPos)
  {
    positionLeft = -encLeft.read()/(cpr/res);  
    positionRight = encRight.read()/(cpr/res); 

    if (positionLeft <= LeftFinalPos){
        motor1.brake();
    }

    if (positionRight <= RightFinalPos){
        motor2.brake();
    }       
  }
}

void turnLeft(int deg)
{ 
  int steps = axel/radius/720*deg*res;
  long LeftFinalPos = positionLeft + steps;
  long RightFinalPos = positionRight - steps;
  motor1.drive(255);
  motor2.drive(-255);

  while (positionLeft < LeftFinalPos || positionRight > RightFinalPos)
  {
    positionLeft = -encLeft.read()/(cpr/res);  
    positionRight = encRight.read()/(cpr/res); 

    if (positionLeft >= LeftFinalPos){
        motor1.brake();
    }

    if (positionRight <= RightFinalPos){
        motor2.brake();
    }       
  }
}

void turnRight(int deg)
{
  int steps = axel/radius/720*deg*res;
  long LeftFinalPos = positionLeft - steps;
  long RightFinalPos = positionRight + steps;
  
  motor1.drive(-255);
  motor2.drive(255);

  while (positionLeft > LeftFinalPos || positionRight < RightFinalPos)
  {
    positionLeft = -encLeft.read()/(cpr/res);  
    positionRight = encRight.read()/(cpr/res); 

    if (positionLeft <= LeftFinalPos){
        motor1.brake();
    }

    if (positionRight >= RightFinalPos){
        motor2.brake();
    }       
  }
}
