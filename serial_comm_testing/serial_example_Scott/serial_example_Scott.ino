void setup() {
  Serial.begin(9600);


  // we had it up here 
  // and there was an interrupt
  // serial interrupt

  
}
void loop() {
    // these could be global variables or however you want to do it
    
    String fire_cmd = "f";   
    int rot_on, rot_dir, rot_steps, rot_delay, pit_on, pit_dir, pit_steps, pit_delay;
    
    // again these could be declared before setup if you want, I just put them here because I think this will work ... ?


  // SCOTT VERY IMPORTANT
  // THE SERIAL READ FUNCTION MUST BE HAPPENING ALL THE TIME, I'M NOT SURE HOW TO DO IT, BUT EACH TIME YOU GET A SERIAL IT SHOULD BE CHECKING THE VALUES COMMANDED
  // This is so that it can react instantly to when the targeting program sends it a command, it will instantly slow down/speed up the steppers.  
  /* EX: loop {
      check serial function
      move steppers with specified variables and keep them moving until we tell them to stop with command
  }
  */

  


// you can turn this if statement into a function if you want.  Something like def checkserial : { return all the command variables}
  if (Serial.available() > 0) {

    // reads the serial line, parsing commands into strings
    String r_on = Serial.readStringUntil(',');
    String r_dir = Serial.readStringUntil(',');
    String r_steps = Serial.readStringUntil(',');
    String r_delay = Serial.readStringUntil(',');
    
    String p_on = Serial.readStringUntil(',');
    String p_dir = Serial.readStringUntil(',');
    String p_steps = Serial.readStringUntil(',');
    String p_delay = Serial.readStringUntil('\n');

    // checks to see if we've changed to fire mode (r_on or p_on is sent an 'f' instead of a 0 or 1)
    if (r_on.equals(fire_cmd) || p_on.equals(fire_cmd)) {
      
       // send fire command here to servo //
       // maybe a function // 
       // everything pauses while this happens // 
       
    } else {
      // convert all the string commands to integers
      rot_on = r_on.toInt();
      rot_dir = r_dir.toInt();
      rot_steps = r_steps.toInt();
      rot_delay = r_delay.toInt();
      
      pit_on = p_on.toInt();
      pit_dir = p_dir.toInt();
      pit_steps = p_steps.toInt();
      pit_delay = p_delay.toInt();      
    }
  }


    if (rot_steps == 0) {
      // make rot stepper run with the set delay (this is what will happen most of the time)
    } else {
      // go this many steps, might be a good idea to delay() until steps are done too
    }


    if (pit_steps == 0) {
      // make pit stepper run with the set delay (this is what will happen most of the time)
    } else {
      // go this many steps, might be a good idea to delay() until steps are done too
    }
    

    

    // rotation command string = on/off(1 or 0), direction(1 or 0), #steps(if 0, = while), 1/velocity(int delay in us),
    // pitch command string    = on/off(1 or 0), direction (1 or 0), #steps(if 0, = while), 1/velocity(int delay in us) '\n'

    // total commmand string   = 'rot_on(0 or 1),rot_dir(0 or 1),rot_steps(0 or #steps),rot_delay(#us),pit_on(0 or 1),pit_dir(0 or 1),pit_steps(0 or #steps),pit_delay(#us)\n'

  
}
