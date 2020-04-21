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
long pit_steps_from_home = 1;
long num_servo_pulls = 1;


void setup() {
  // Start I2C Bus as Slave

  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
//  Wire.onRequest(sendData);


//  Serial.begin(9600);
//  Serial.println("starting test");
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
//    Serial.print("byte 1 = ");
//    Serial.println(c1);

    int field = c1;
//    Serial.print("field = ");
//    Serial.println(c1);
    cmd_str[i] = field;
  }
  
//  Serial.print("command string = ");
//  for (int n = 0; n < 9; n++) {
//    Serial.println(cmd_str[n]);
//
//  }
  fire      = cmd_str[0];
//  Serial.print("fire = ");
//  Serial.println(fire);
  rot_on    = cmd_str[1];
//  Serial.print("rot_on = ");
//  Serial.println(rot_on);
  rot_dir   = cmd_str[2];
//  Serial.print("rot_dir = ");
//  Serial.println(rot_dir);
  rot_steps = cmd_str[3];
//  Serial.print("rot_steps = ");
//  Serial.println(rot_steps);
  rot_delay = 100*cmd_str[4];
//  Serial.print("rot_delay = ");
//  Serial.println(rot_delay);
  pit_on    = cmd_str[5];
//  Serial.print("pit_on = ");
//  Serial.println(pit_on);
  pit_dir   = cmd_str[6];
//  Serial.print("pit_dir = ");
//  Serial.println(pit_dir);
  pit_steps = cmd_str[7];
//  Serial.print("pit_steps = ");
//  Serial.println(pit_steps);
  pit_delay = 100*cmd_str[8];
//  Serial.print("pit_delay = ");
//  Serial.println(pit_delay);
}


void sendEvent()
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
