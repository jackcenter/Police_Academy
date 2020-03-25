//Keep the pin 7,2,4 floating ALWAYS 
//ALWAYS 
//ALWAYS
#include <Wire.h>
// #include <ros.h>
// #include <ros/time.h>
// #include <sensor_msgs/Range.h>
const int pingPin1 = A0;
const int trigPin1 = 4;           //connects to the trigger pin on the distance sensor       
const int echoPin1 = A1; 
const int pingPin2 = A2;
const int trigPin2 = 4;           //connects to the trigger pin on the distance sensor       
const int echoPin2 = A3; 
const boolean CENTIMETERS = true;
const boolean INCHES = false;
float distance1 = 0;
float distance2 = 0;
    
void setup()
{
  // Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
  
  //Ultrasonic Right
  
  pinMode(trigPin1, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin1, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor
  
  //Ultrasonic Left
  pinMode(trigPin2, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin2, INPUT);   
}

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance(int trigPin, int echoPin) 
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time
  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor
  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  delay(200);
  return calculatedDistance;              //send back the distance that was calculated
}

//float gd1() {
//  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
//  float calculatedDistance;         //variable to store the distance calculated from the echo time
//  //send out an ultrasonic pulse that's 10ms long
////  digitalWrite(trigPin2, HIGH);
//  delayMicroseconds(10); 
////  digitalWrite(trigPin2, LOW);
//  echoTime = pulseIn(echoPin2, HIGH);      //use the pulsein command to see how long it takes for the
//                                          //pulse to bounce back to the sensor
//  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
//  return calculatedDistance;              //send back the distance that was calculated
//}

void loop() 
{
  Serial.print(getDistance(trigPin1, echoPin1));
  Serial.print(" / ");
  Serial.println(getDistance(trigPin2, echoPin2));
  delay(100);
 
//  //Ultrasonic Left
//  distace2=getDistance();
//  Serial.println(distance2)
//  delay(500);    
}
