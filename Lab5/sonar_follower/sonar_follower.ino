#include <SparkFun_TB6612.h>

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

#define pingPinLeft A0
#define echoPinLeft A1
#define pingPinRight A2
#define echoPinRight A3  
#define trigPin  4

// Ultrasonic Settings
const float inchSound = 148;    
float distanceLeft = 0;
float distanceRight = 0;
float chassisDist = 4;
float maxDistanceReading = 10;

// Motor Settings
const int offsetLeft = -1;
const int offsetRight = 1;
Motor motorRight = Motor(AIN1, AIN2, PWMA, offsetLeft, STBY);
Motor motorLeft = Motor(BIN1, BIN2, PWMB, offsetRight, STBY);
int leftBaseSpeed = 108;
int rightBaseSpeed = 117;
int leftSpeed = leftBaseSpeed;
int rightSpeed = rightBaseSpeed;
float error = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);     //the trigger pin will output pulses of electricity 
  pinMode(echoPinLeft, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor
  pinMode(echoPinRight, INPUT);    
}

void loop() {
  getDistance(distanceLeft, distanceRight);
  // error = getError(distanceLeft,distanceRight);
  adjustSpeed(leftSpeed, rightSpeed, distanceLeft, distanceRight);

  Serial.print("Line 1: ");
  Serial.print(distanceLeft);
  Serial.print(" / ");
  Serial.print(distanceRight);
  Serial.print(" / ");
  Serial.println(error);

  Serial.print("Line 2: ");
  Serial.print(leftSpeed);
  Serial.print(" / ");
  Serial.println(rightSpeed);
  Serial.println();

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  delay(100);
}

void getDistance(float &distLeft, float &distRight) 
{
  float echoTimeLeft;                   //variable to store the time it takes for a ping to bounce off an object
  float echoTimeRight;
  float calculatedDistance;         //variable to store the distance calculated from the echo time
  
  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delay(10); 
  digitalWrite(trigPin, LOW);
  echoTimeLeft = pulseIn(echoPinLeft, HIGH);      //use the pulsein command to see how long it takes for the pulse to bounce back to the sensor

  delay(100);

  digitalWrite(trigPin, HIGH);
  delay(10); 
  digitalWrite(trigPin, LOW);
  echoTimeRight = pulseIn(echoPinRight, HIGH);

  distLeft = echoTimeLeft / inchSound - chassisDist;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  distRight = echoTimeRight / inchSound - chassisDist;
  delay(200);
}

void adjustSpeed(int &spdLeft, int &spdRight, float distLeft, float distRight)
{
  char mode;
  bool leftData = validate_sensor_reading(distLeft);
  bool rightData = validate_sensor_reading(distRight);

  // mode A - both sensors have readings
  // mode B - neither sensor has readings
  // mode L - left sensor has readings
  // mode R - right sensor has readings

  // DETERMINE MODE
  if (leftData == true && rightData == true){
    mode = 'A';
  }
  else if (leftData == true && rightData == false){
    mode = 'L';
  }
  else if (leftData == false && rightData == true){
    mode = 'R';
  }
  else {
    mode = 'B';
  } 

  Serial.println(mode);
  
  switch (mode){
      case 'A':
        break;
      case 'B':
        mode_B(spdLeft, spdRight);
        break;
      case 'L':
        error = getError(distLeft);
        mode_L(spdLeft, error);
        break;
      case 'R':
        error = getError(distRight);
        mode_L(spdRight, error);
        break;
  }
} 

void mode_B(int &spdLeft, int &spdRight)
{
  spdLeft = leftBaseSpeed;
  spdRight = rightBaseSpeed;
}

void mode_L(int &spdLeft, float error)
{
  if (error < -1)
  {
    spdLeft = leftBaseSpeed + 50;
  }

  else if (error > 2)
  {
    spdLeft = leftBaseSpeed - 35;
  }

  else
  {
    spdLeft = leftBaseSpeed;
  }
}

void mode_R(int &spdRight, float error)
{
  if (error < -1)
  {
    spdRight = rightBaseSpeed + 50;
  }

  else if (error > 2)
  {
    spdRight = rightBaseSpeed - 35;
  }

  else
  {
    spdRight = rightBaseSpeed;
  }
}

bool validate_sensor_reading(float data)
{
  if (abs(data) > maxDistanceReading)
  {
    return false;
  }

  else
  {
    return true;
  }
}

float getError(float dist)
{
  // JUST FOLLWING SENSOR
  // negative error is too close
  // positive error is too far
  return dist - 4;
}
