
// constants won't change. They're used here to set pin numbers:
const int Z1_BUTTON = 10;    // the number of the pushbutton pin
const int Z2_BUTTON = 11;    // the number of the pushbutton pin
const int Y1_BUTTON = 12;    // the number of the pushbutton pin
const int Y2_BUTTON = 13;    // the number of the pushbutton pin
const int Y_STEP_PIN = 3; 
const int Y_DIR_PIN = 4; 
const int Z_STEP_PIN = 8; 
const int Z_DIR_PIN = 9; 
int Y1_BUTTON_STATE;             // the current reading from the input pin
int Y1_BUTTON_LAST_STATE = LOW;   // the previous reading from the input pin
int Y2_BUTTON_STATE;             
int Y2_BUTTON_LAST_STATE = LOW;   
int Z1_BUTTON_STATE;             // the current reading from the input pin
int Z1_BUTTON_LAST_STATE = LOW;   // the previous reading from the input pin
int Z2_BUTTON_STATE;             
int Z2_BUTTON_LAST_STATE = LOW;   
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime_Y1 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_Y2 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_Z1 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_Z2 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
void setup() {
 pinMode(Y1_BUTTON, INPUT);
 pinMode(Y2_BUTTON, INPUT);
 pinMode(Z1_BUTTON, INPUT);
 pinMode(Z2_BUTTON, INPUT);
 
 pinMode(Y_STEP_PIN,OUTPUT); 
 pinMode(Y_DIR_PIN,OUTPUT);
 pinMode(Z_STEP_PIN,OUTPUT); 
 pinMode(Z_DIR_PIN,OUTPUT);
 Serial.begin(9600);
 Serial.println("Let's Get Steppin'");
}
void loop() {
  // read the state of the switch into a local variable:
  int Y1_BUTTON_reading = digitalRead(Y1_BUTTON);
  int Y2_BUTTON_reading = digitalRead(Y2_BUTTON);
  int Z1_BUTTON_reading = digitalRead(Z1_BUTTON);
  int Z2_BUTTON_reading = digitalRead(Z2_BUTTON);
  
  if (Y1_BUTTON_reading != Y1_BUTTON_LAST_STATE) {
    // reset the debouncing timer
    lastDebounceTime_Y1 = millis();
  }
  if (Y2_BUTTON_reading != Y2_BUTTON_LAST_STATE) {
    // reset the debouncing timer
    lastDebounceTime_Y2 = millis();
  }
  if (Z1_BUTTON_reading != Z1_BUTTON_LAST_STATE) {
    // reset the debouncing timer
    lastDebounceTime_Z1 = millis();
  }
  if (Z2_BUTTON_reading != Z2_BUTTON_LAST_STATE) {
    // reset the debouncing timer
    lastDebounceTime_Z2 = millis();
  }
  if ((millis() - lastDebounceTime_Y1) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (Y1_BUTTON_reading != Y1_BUTTON_STATE) {
      Y1_BUTTON_STATE = Y1_BUTTON_reading;
      // only toggle the LED if the new button state is HIGH
      if (Y1_BUTTON_STATE == HIGH) {
         Serial.println("Y - ");
         digitalWrite(Y_DIR_PIN    , HIGH);
          
        for(int a=0;a<400;a++){
            digitalWrite(Y_STEP_PIN   , HIGH);
            delayMicroseconds(1000);
            digitalWrite(Y_STEP_PIN   , LOW);
          }   
      }
    }
  }
  if ((millis() - lastDebounceTime_Y2) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (Y2_BUTTON_reading != Y2_BUTTON_STATE) {
      Y2_BUTTON_STATE = Y2_BUTTON_reading;
      // only toggle the LED if the new button state is HIGH
      if (Y2_BUTTON_STATE == HIGH) {
         Serial.println("Y + ");
         digitalWrite(Y_DIR_PIN    , LOW);
          
        for(int b=0;b<400;b++){
            digitalWrite(Y_STEP_PIN   , HIGH);
            delayMicroseconds(1000);
            digitalWrite(Y_STEP_PIN   , LOW);
          }   
      }
    }
  }
   if ((millis() - lastDebounceTime_Z1) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (Z1_BUTTON_reading != Z1_BUTTON_STATE) {
      Z1_BUTTON_STATE = Z1_BUTTON_reading;
      // only toggle the LED if the new button state is HIGH
      if (Z1_BUTTON_STATE == HIGH) {
         Serial.println("Z - ");
         digitalWrite(Z_DIR_PIN    , HIGH);
          
        for(int c=0;c<100;c++){
            digitalWrite(Z_STEP_PIN   , HIGH);
            delayMicroseconds(5000);
            digitalWrite(Z_STEP_PIN   , LOW);
          }   
      }
    }
  }
  if ((millis() - lastDebounceTime_Z2) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (Z2_BUTTON_reading != Z2_BUTTON_STATE) {
      Z2_BUTTON_STATE = Z2_BUTTON_reading;
      // only toggle the LED if the new button state is HIGH
      if (Z2_BUTTON_STATE == HIGH) {
         Serial.println("Z + ");
         digitalWrite(Z_DIR_PIN    , LOW);
          
        for(int d=0;d<100;d++){
            digitalWrite(Z_STEP_PIN   , HIGH);
            delayMicroseconds(5000);
            digitalWrite(Z_STEP_PIN   , LOW);
          }   
      }
    }
  }
  // set the LED:
  //digitalWrite(ledPin, ledState);
       
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  Y1_BUTTON_LAST_STATE = Y1_BUTTON_reading;
  Y2_BUTTON_LAST_STATE = Y2_BUTTON_reading;
  Z1_BUTTON_LAST_STATE = Z1_BUTTON_reading;
  Z2_BUTTON_LAST_STATE = Z2_BUTTON_reading;
}
