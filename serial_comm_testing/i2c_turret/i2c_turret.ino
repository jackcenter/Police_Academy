#include <Wire.h>

byte slave_address = 8;

void setup() {
  // Start I2C Bus as Slave

  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
//  Wire.onRequest(sendData);


  Serial.begin(9600);
  Serial.println("starting test");
}

void loop() {

}

void receiveEvent() {
//  int numOfBytes = Wire.available();
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
  
  Serial.print("command string = ");
  for (int n = 0; n < 9; n++) {
    Serial.println(cmd_str[n]);

  }
    
  delay(50);

// HERE IS WHERE WE'LL NEED TO WRITE A FUNCTION THAT SUCKS OUT THE RECIEVE EVENT 
// COMMAND STRING AND CONVERTS IT TO USABLE VARIABLES
}





//void sendEvent()
//{
//  int numOfBytes = Wire.available();
//  Serial.print("len: ");
//  Serial.println(numOfBytes);
//  byte side = Wire.read();
//  Serial.print("Send: ");
//  Serial.println(side);
//  if (side == 0){
//      buffer.longNumber = rightPos;     
//      Wire.write(buffer.longBytes, 4);
//      buffer.longNumber = leftPos;
//      Wire.write(buffer.longBytes, 4);
//  }
//  
//  else if (side == 1){
//      buffer.longNumber = leftPos;
//      Wire.write(buffer.longBytes, 4);
//  }
//  
//  
//  Serial.print(" Right position: ");
//  Serial.println(rightPos);
//  Serial.print(" Left position:  ");
//  Serial.println(leftPos);
//  Serial.println();
//}
