void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    String r_cmd = Serial.readStringUntil(',');
    String p_cmd = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.print(r_cmd);
    Serial.print("   and:   ");
    Serial.println(p_cmd);
  }
}
