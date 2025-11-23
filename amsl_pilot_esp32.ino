#include <ESP32Servo.h>

const int SERVO1 = 2; //Left, or Thruster
const int SERVO2 = 3; //Right, or Rudder
const int RC = 4;     //RC PPM input

const unsigned long WATCHDOG_DELAY = 5000;
const unsigned long STATUS_DELAY = 1000;

// Servos
Servo servo1;
Servo servo2;

//RC Override
unsigned long usLastPulse;
unsigned long usPrevPulse;
unsigned long usDelta;
unsigned long pulseLengths[6];
int pulseCount = 0;

// State
char state[256];
unsigned long servo_1_us = 1500;
unsigned long servo_2_us = 1500;
bool rc_override = false;
char cmd[64];
unsigned int cmd_i = 0;
unsigned long last_status = millis();
unsigned long last_servo_cmd = millis();
String control_state = "<>";


/** setServos
      Update servo PWM to latest requested
*/
void setServos(String id) {
  control_state = id;
  servo1.write(servo_1_us);
  servo2.write(servo_2_us);
  last_servo_cmd = millis();
}
/** status
      Generate a serial message with current system status
*/
void status() {
  sprintf(state, "[%05d] S1:%04d S2:%04d %2s",millis()%100000,servo_1_us,servo_2_us,control_state.c_str());
  Serial.println(state);
  last_status = millis();  
}
/** parseCmd
      parse latest serial cmd for validity and update servo request
*/
bool parseCmd(){
  unsigned long thrust_sp;
  unsigned long rudder_sp;
  int count = sscanf(cmd, "$CMD,%i,%i",&thrust_sp, &rudder_sp);
    if (count == 2) {
      servo_1_us = thrust_sp;
      servo_2_us = rudder_sp;
      return true;
    }
  return false;
}
/** rcInterrupt
      Interrupt routine for RC input
*/
void rcInterrupt() {
  usPrevPulse = usLastPulse;
  usLastPulse = micros();
  usDelta = usLastPulse - usPrevPulse;

  if (usDelta > 2100){
    pulseCount = 0;
  }
  else {
    pulseLengths[pulseCount] = usDelta;
    pulseCount++;
  }
}


/** setup
      configure system 
*/
void setup() {
  Serial.begin(9600);
  delay(1000);
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo1.setPeriodHertz(50);
	servo1.attach(SERVO1, 1300, 1700);
  servo2.setPeriodHertz(50);
	servo2.attach(SERVO2, 1300, 1700);
  
  // Setup ESC
  Serial.println("Initialising ESC");
  servo1.write(1500);
  servo2.write(1500);
  delay(5000);
  Serial.println("End Setup");

  attachInterrupt(digitalPinToInterrupt(RC), rcInterrupt, RISING);
}


/** Main program loop
*/
void loop() {
  
  // Check RC override
  //  if rc_override enabled, set servos
  if (pulseLengths[4] == 2000) {
    rc_override = true;
    servo_1_us = pulseLengths[1];
    servo_2_us = pulseLengths[0];
    setServos("RC");
  }
  else
    rc_override = false;
  

  // Check for serial command
  //  if not in rc_override and command parses, set servos
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n'){
      if (!rc_override & parseCmd())
        setServos("UI");
      cmd_i = 0;
      cmd[0] = '\0';
      break;
    }else {
      cmd[cmd_i] = inChar;
      cmd_i++;
      if (cmd_i > 63){
        cmd_i = 0;
        Serial.println("SERIAL OVERRUN");
        cmd[0] = '\0';
        break;
      } 
    }
  }

  // Watchdog
  //  if too much time has elapsed since rc or user command
  //    set servos to stop
  if (millis() - last_servo_cmd > WATCHDOG_DELAY) {
    servo_1_us = 1500;
    servo_2_us = 1500;
    setServos("WD");
  }
  //setServos("TS");
  //Update Status
  //  periodically send a status message
  if (millis() - last_status > STATUS_DELAY) {
    status();
  }

  delay(100);
}
