#include <Wire.h>


union Buffer
{
    long longNumber;
    byte longBytes[4];
};

byte slave_address = 8;
Buffer buffer;

int fire;
int rot_on;
int rot_dir;
int rot_steps;
int rot_delay;
int pit_on;
int pit_dir;
int pit_steps;
int pit_delay;
long rot_steps_from_home = 1;
long pit_steps_from_home = 2;
long num_servo_pulls = 3;


void setup() {
  // Start I2C Bus as Slave

  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendData);


}

void loop() {

}

void receiveEvent() {
  int numOfBytes = Wire.available();
  int cmd_str[9];
  // there are 9 command string fields and each field is 1 byte

  byte whocares = Wire.read();  //throw away first byte because it ends up being an extra 0 for some reason
  for (int i = 0; i < 10; i++) {    // master sends 18 bytes or 9 fields at 2 bytes each
    byte c1 = Wire.read();    // receive a byte and it's already an int!  Easy!
    int field = c1;
    cmd_str[i] = field;
  }
  
  fire      = cmd_str[0];
  rot_on    = cmd_str[1];
  rot_dir   = cmd_str[2];
  rot_steps = cmd_str[3];
  rot_delay = 100*cmd_str[4];
  pit_on    = cmd_str[5];
  pit_dir   = cmd_str[6];
  pit_steps = cmd_str[7];
  pit_delay = 100*cmd_str[8];
}




void sendData()
{
  int numOfBytes = Wire.available();
  Wire.read();
//  Serial.print("rot_steps_from_home = ");
//  Serial.println(rot_steps_from_home);
  buffer.longNumber = rot_steps_from_home;     
  Wire.write(buffer.longBytes, 4);
//  Serial.print("pit_steps_from_home = ");
//  Serial.println(pit_steps_from_home);
  buffer.longNumber = pit_steps_from_home;     
  Wire.write(buffer.longBytes, 4);  
//  Serial.print("num_servo_pulls = ");
//  Serial.println(num_servo_pulls);
  buffer.longNumber = num_servo_pulls;
  Wire.write(buffer.longBytes, 4);
}
