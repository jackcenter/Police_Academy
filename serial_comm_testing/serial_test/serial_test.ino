void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    String fire_cmd = "f";   // these could be global variables or however you want to do it
    float rot_cmd = 0;
    float pit_cmd = 0;
    String r_cmd = Serial.readStringUntil(',');
    String p_cmd = Serial.readStringUntil('\n');
    
    if (r_cmd.equals(fire_cmd) || p_cmd.equals(fire_cmd)) {
       // send fire command here to servo
    } else {
      rot_cmd = r_cmd.toFloat();
      pit_cmd = p_cmd.toFloat();
    }
    }
  // go to home, reverse steps ? 
  // go home .. 

      rot_cmd = [on/off (1 or 0), direction (0 or 1), #steps (if 0, = while), 1/velocity (int gap between high and low pulse #us)]
      pit_cmd = [on/off (1 or 0), direction (0 or 1), #step (if 0, = while), 1/velocity (int gap between high and low pulse #us)]

  
}
