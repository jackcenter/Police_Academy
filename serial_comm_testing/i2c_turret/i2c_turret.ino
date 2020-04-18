#include <Wire.h>

#define LED_PIN 13

byte slave_address = 8;
//byte CMD_ON = 0x00;
//byte CMD_OFF = 0x01;

void setup() {
  // Start I2C Bus as Slave
  // Start I2C Bus as Slave
  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
//  Wire.onRequest(sendEvent);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  

  Serial.begin(9600);
}

void loop() {

}


void receiveEvent()
{
//  int numOfBytes = Wire.available();
  int cmd_str[9];
  // there are 9 command string fields and each field is 2 bytes

  for (i = 0; i < 10; ++i) {    // master sends 18 bytes or 9 fields at 2 bytes each
    byte c1 = Wire.read();    // receive a byte as character? or can i do int?
    byte c2 = Wire.read();    // recieve second byte as the other hald of the cmd field
    // convert bytes to int
    Serial.print("byte 1 = ");
    Serial.println(c1);
    Serial.print("byte 2 = ");
    Serial.println(c2);

// instead convert to word = two bytes and is equivalent to unsigned int ...?
    int field = word(c1, c2);  // may have to switch order of these two depending on whether bytes are ordered big or little
    Serial.print("field word = ");
    cmd_str[i] = field;
    
    
//  unsigned char tmp[2] = {c1, c2};
// 
//  unsigned long sID = ((unsigned long)(tmp[0]) << 24)
//           | ((unsigned long)(tmp[1]) << 16)
//           | ((unsigned long)(tmp[2]) << 8)
//           | tmp[3];
//  Serial.print(sID, DEC);

  }
  Serial.print("command string = ");
  Serial.println(cmd_str);
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
