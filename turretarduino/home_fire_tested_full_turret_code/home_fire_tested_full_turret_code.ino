

#include <Servo.h>
#include <Wire.h>


union Buffer{
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

int home_cmd = 0;
long rot_steps_from_home;
long pit_steps_from_home;
long num_servo_pulls = 0;//shot counter
long pit_break = 0;
long rot_break = 0;


Servo myservo;      // changed name cwm

// constants won't change. ThePITCH're used here to set pin numbers:
const int ROT1_BUTTON = 10;    // the number of the pushbutton pin
const int ROT2_BUTTON = 11;    // the number of the pushbutton pin
const int PITCH1_BUTTON = 12;    // the number of the pushbutton pin
const int PITCH2_BUTTON = 13;    // the number of the pushbutton pin
const int PITCH_STEP_PIN = 4; 
const int PITCH_DIR_PIN = 3; 


const int ROT_STEP_PIN = 9; 
const int ROT_DIR_PIN = 8; 


const int ROT_HOME_PIN = 7;
const int PITCH_MIN_PIN = 6;
const int PITCH_MAX_PIN = 5;

const int pwm = 2;

unsigned long lastDebounceTime_ROTHOME = 0;  // the last time the output pin was toggled
int ROT_HOME_PIN_val= digitalRead(ROT_HOME_PIN);
int PITCH_MAX_PIN_val= digitalRead(PITCH_MAX_PIN);
int PITCH_MIN_PIN_val= digitalRead(PITCH_MIN_PIN);


int ROT_HOME_PIN_STATE;
int ROT_HOME_PIN_LAST_STATE = LOW;
int PITCH_MIN_PIN_STATE;
int PITCH_MIN_PIN_LAST_STATE = LOW;
int PITCH_MAX_PIN_STATE;
int PITCH_MAX_PIN_LAST_STATE = LOW;

int PITCH1_BUTTON_STATE;             // the current reading from the input pin
int PITCH1_BUTTON_LAST_STATE = LOW;   // the previous reading from the input pin
int PITCH2_BUTTON_STATE;             
int PITCH2_BUTTON_LAST_STATE = LOW;   
int ROT1_BUTTON_STATE;             // the current reading from the input pin
int ROT1_BUTTON_LAST_STATE = LOW;   // the previous reading from the input pin
int ROT2_BUTTON_STATE;             
int ROT2_BUTTON_LAST_STATE = LOW;   

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quicklPITCH become a bigger number than can be stored in an int.
unsigned long lastDebounceTime_PITCH1 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_PITCH2 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_ROT1 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_ROT2 = 0;  // the last time the output pin was toggled

unsigned long debouncedelay = 50;    // the debounce time; increase if the output flickers
int counter=0;

void setup() {
Wire.begin(slave_address);
Wire.onReceive(receiveEvent);
Wire.onRequest(sendData);

  
 pinMode(PITCH1_BUTTON, INPUT);
 pinMode(PITCH2_BUTTON, INPUT);
 pinMode(ROT1_BUTTON, INPUT);
 pinMode(ROT2_BUTTON, INPUT);
 
 pinMode(PITCH_STEP_PIN,OUTPUT); 
 pinMode(PITCH_DIR_PIN,OUTPUT);
 pinMode(ROT_STEP_PIN,OUTPUT); 
 pinMode(ROT_DIR_PIN,OUTPUT);
 pinMode(ROT_HOME_PIN, INPUT_PULLUP);
 pinMode(PITCH_MAX_PIN, INPUT_PULLUP);
 pinMode(PITCH_MIN_PIN, INPUT_PULLUP);
 
 myservo.attach(46);  // attaches the servo on pin 9 to the servo object
 myservo.write(118);
 
 Serial.begin(9600);
 pinMode(pwm, OUTPUT);
 digitalWrite(pwm, LOW);
   

}

void loop() {
  // read the state of the switch into a local variable:
//  int PITCH1_BUTTON_reading = digitalRead(PITCH1_BUTTON);
//  int PITCH2_BUTTON_reading = digitalRead(PITCH2_BUTTON);
//  int ROT1_BUTTON_reading = digitalRead(ROT1_BUTTON);
//  int ROT2_BUTTON_reading = digitalRead(ROT2_BUTTON);     //commented these out
  
//  int PITCH_MIN_PIN_val = digitalRead(PITCH_MIN_PIN);
//  int PITCH_MAX_PIN_val = digitalRead(PITCH_MAX_PIN);     // commented this out
 /*
int fire;
int rot_on;
int rot_dir;
int rot_steps;
int rot_delay;
int pit_on;
int pit_dir;
int pit_steps;
int pit_delay;

int home_cmd = 0;
long rot_steps_from_home;
long pit_steps_from_home;
long num_servo_pulls;//shot counter
*/ 

if (fire==1) {
  firesequence();
  delay(1000);
}

else {                            // needs regular else statement here
  if (rot_on==2  ||  pit_on==2){
    movetohome();
  }
  else {
    if (rot_on==1) {
    rotmove();  
    if (rot_break == 1) {
      rot_on == 0;
    }
    }
  
  if (pit_on==1) {
    pitmove();  
    if (pit_break == 1) {
      pit_on == 0;
    }
  }
}
  
}


//  PITCH1_BUTTON_LAST_STATE = PITCH1_BUTTON_reading;           // removed button states
//  PITCH2_BUTTON_LAST_STATE = PITCH2_BUTTON_reading;
//  ROT1_BUTTON_LAST_STATE = ROT1_BUTTON_reading;
//  ROT2_BUTTON_LAST_STATE = ROT2_BUTTON_reading;
  
} //end LOOP
void firesequence(){
      digitalWrite(pwm, HIGH); 
      delay(4000);
     //myservo.write(36); //pull trigger back completely
      myservo.write(0); //pull trigger back completely with dart?

      delay(1000);
     myservo.write(118); 
      digitalWrite(pwm, LOW);
      num_servo_pulls+=1;
  
}
void movetohome(){
       
        digitalWrite(PITCH_DIR_PIN    , LOW); 
        for(int a=0;a<12000;a++){
           int PITCH_MIN_PIN_val= digitalRead(PITCH_MIN_PIN);
           if (PITCH_MIN_PIN_val == HIGH) {
            break;
           }
            digitalWrite(PITCH_STEP_PIN   , HIGH);
            delayMicroseconds(1000);
            digitalWrite(PITCH_STEP_PIN   , LOW);
          }
        delay(500);
        digitalWrite(PITCH_DIR_PIN    , HIGH);
        for(int b=0;b<300;b++){
           int PITCH_MAX_PIN_val= digitalRead(PITCH_MAX_PIN);
           if (PITCH_MAX_PIN_val == HIGH) {
            break;
           }
            digitalWrite(PITCH_STEP_PIN   , HIGH);
            delayMicroseconds(1000);
            digitalWrite(PITCH_STEP_PIN   , LOW);
        }   
          
       digitalWrite(ROT_DIR_PIN,HIGH);
       for(int c=0;c<400;c++){
           int ROT_HOME_PIN_val= digitalRead(ROT_HOME_PIN);
           if (ROT_HOME_PIN_val == HIGH) {
              break; 
           }
           else if (ROT_HOME_PIN_val == LOW) {
              digitalWrite(ROT_STEP_PIN   , HIGH);
              delayMicroseconds(10000); //5000 was good, but fast
              digitalWrite(ROT_STEP_PIN   , LOW);
           }
           }
       delay(500);
       digitalWrite(ROT_DIR_PIN,LOW);
   
       for(int c=0;c<800;c++){
           int ROT_HOME_PIN_val= digitalRead(ROT_HOME_PIN);
           if (ROT_HOME_PIN_val == HIGH) {
              break; 
           }
           else if (ROT_HOME_PIN_val == LOW) {
              digitalWrite(ROT_STEP_PIN   , HIGH);
              delayMicroseconds(10000);
              digitalWrite(ROT_STEP_PIN   , LOW);
           }
           }
        delay(500);
        digitalWrite(ROT_DIR_PIN    , HIGH);
        for(int c=0;c<65;c++){
            digitalWrite(ROT_STEP_PIN   , HIGH);
            delayMicroseconds(10000);
            digitalWrite(ROT_STEP_PIN   , LOW);
          }
       rot_steps_from_home=0;
       pit_steps_from_home=0;
}



void pitmove(){
/*
int pit_on;
int pit_dir;
int pit_steps;
int pit_delay;
 */ 
        Serial.print("pit_steps = ");
        Serial.println(pit_steps);
        if (pit_dir==0){
        digitalWrite(PITCH_DIR_PIN    , LOW);
        }
        else if (pit_dir==1) {
        digitalWrite(PITCH_DIR_PIN    , HIGH);
        }
        
        if (pit_steps==0) {
          Serial.println("I'm in pit vel command section!");
            int PITCH_MIN_PIN_val= digitalRead(PITCH_MIN_PIN);
            int PITCH_MAX_PIN_val= digitalRead(PITCH_MAX_PIN);
            if (PITCH_MIN_PIN_val == HIGH) {
            pit_break = 1;                                                        // added pitch break
//            break;                                                              set pitch break variable and then check in loop
           } else {
            pit_break = 0;
           }
           if (PITCH_MAX_PIN_val == HIGH) {
            pit_break = 1;                                                        // added pitch break
//            break;                                                              set pitch break variable and then check in loop
           } else {
            pit_break = 0;
           }
            digitalWrite(PITCH_STEP_PIN   , HIGH);
            delayMicroseconds(pit_delay);//1000 was good for testing
            digitalWrite(PITCH_STEP_PIN   , LOW);
          if (pit_dir==0) {
           pit_steps_from_home-=1;
           }
          else if (pit_dir==1) {
           pit_steps_from_home+=1;
           } 
          
        }
        else {                                                   // removed else if
          for(int a=0;a<pit_steps;a++){
            Serial.println("I'm in pit step command section!");
            int PITCH_MIN_PIN_val= digitalRead(PITCH_MIN_PIN);
            int PITCH_MAX_PIN_val= digitalRead(PITCH_MAX_PIN);
            if (PITCH_MIN_PIN_val == HIGH) {
              pit_break = 0;
              break;
           }
           if (PITCH_MAX_PIN_val == HIGH) {
              pit_break = 0;
              break;
           }
            digitalWrite(PITCH_STEP_PIN, HIGH);
            delayMicroseconds(pit_delay);   //1000 was good for testing   // this should be larger right?  1000 would be weird ... ?
            digitalWrite(PITCH_STEP_PIN, LOW);
          }                                                            
          if (pit_dir==0) {
          pit_steps_from_home -= pit_steps;
           }
          else if (pit_dir==1) {
           pit_steps_from_home += pit_steps;
           } 
        }
}

void rotmove(){
/*
 rot_delay = 100*cmd_str[4];   //5,000 is max speed, 25,000 is min
  pit_on    = cmd_str[5];  //0 (off), 1 to enable pitch (If steps=0, run at vel. if steps !=0, move # of steps), 2 to go home   
  pit_dir   = cmd_str[6];  //0 or 1, corresponds with DIR_PIN Low/High
  pit_steps
 */ 
        Serial.print("rot_steps = ");
        Serial.println(rot_steps);
        if (rot_dir==0) {
        digitalWrite(ROT_DIR_PIN    , LOW);
        }
        else if (rot_dir==1) {
        digitalWrite(ROT_DIR_PIN    , HIGH);
        }
        
        if (rot_steps==0){                          // removed_pit_on and changed to rot_steps  
          Serial.println("I'm in vel cmd section!");
          // removed rot min and max checking pins
            digitalWrite(ROT_STEP_PIN   , HIGH);
            delayMicroseconds(rot_delay);//1000 was good for testing
            digitalWrite(ROT_STEP_PIN   , LOW);
          if (rot_dir==0){
           rot_steps_from_home-=1;
           }
          else if (rot_dir==1){
           rot_steps_from_home+=1;
           } 
          
        } else {                                                                // removed else if
          Serial.println("I'm in step cmd section!");
          for(int a=0;a<rot_steps;a++){
          // removed rot min and max checking bc we dont have one
            digitalWrite(ROT_STEP_PIN   , HIGH);
            delayMicroseconds(rot_delay);//1000 was good for testing
            digitalWrite(ROT_STEP_PIN   , LOW);
 
                                                                               // this needs to be lower
            if (rot_dir==0) {
          rot_steps_from_home-=rot_steps;
           }
          else if (rot_dir==1) {
           rot_steps_from_home+=rot_steps;
           }
           
         if (rot_steps_from_home > 600 || rot_steps_from_home < -600) {          // this should be moved into steps command too!
          rot_break = 1;
          break;
        } else {
          rot_break = 0;
          break;
        }           
        }
        }

         if (rot_steps_from_home > 600 || rot_steps_from_home < -600) {          // this should be moved into steps command too!
          rot_break = 1;
        } else {
          rot_break = 0;
        }
           
}

//--------------------------------------------------
void receiveEvent() {
  int numOfBytes = Wire.available();
  int cmd_str[9];
  // there are 9 command string fields and each field is 1 byte

  byte whocares = Wire.read();  //throw awaPITCH first byte because it ends up being an extra 0 for some reason
  for (int i = 0; i < 10; i++) {    // master sends 9 bytes or 9 fields at 2 bytes each
    byte c1 = Wire.read();    // receive a byte and it's alreadPITCH an int!  EasPITCH!
    int field = c1;
    cmd_str[i] = field;
  }
  
  fire      = cmd_str[0];   //0 or 1 to fire
  rot_on    = cmd_str[1];   //0 (off), 1 to enable rot (If steps=0, run at vel. if steps !=0, move # of steps), 2 to go home
  rot_dir   = cmd_str[2];  //0 or 1, corresponds with DIR_PIN Low/High
//  Serial.print(cmd_str[3]);
  rot_steps = 10*cmd_str[3];  //255 is max sent.  Actual range is 0 to 2550 (360deg is approx 800steps) 
                                //If zero, do not move. else, nonzero, move (pass steps to button command)
                                 
  rot_delay = 100*cmd_str[4];    //5,000 is max speed, 25,000 is min
  pit_on    = cmd_str[5];  //0 (off), 1 to enable pitch (If steps=0, run at vel. if steps !=0, move # of steps), 2 to go home   
  pit_dir   = cmd_str[6];  //0 or 1, corresponds with DIR_PIN Low/High
  pit_steps = 10*cmd_str[7];  //255 is max sent.  Actual range is 0 to 2550 (360deg is approx 800steps)  
                               //If zero, do not move. else, nonzero, move (pass steps to button command)
  pit_delay = 100*cmd_str[8];

  //first check fire PITCHes/no, then check home, then check steps rot/pitch
  
  if (rot_on == 2 || pit_on == 2) {
    home_cmd = 1;
  } 
}


void sendData(){
  int numOfBytes = Wire.available();
  Wire.read();
  buffer.longNumber = rot_steps_from_home;     
  Wire.write(buffer.longBytes, 4);
  buffer.longNumber = pit_steps_from_home;     
  Wire.write(buffer.longBytes, 4);  
  buffer.longNumber = num_servo_pulls;
  Wire.write(buffer.longBytes, 4);
  buffer.longNumber = rot_break;
  Wire.write(buffer.longBytes, 4);
  buffer.longNumber = pit_break;
  Wire.write(buffer.longBytes, 4);
}
