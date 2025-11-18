const int SERVO1 = 2; //Left, or Thruster
const int SERVO2 = 3; //Right, or Rudder
const int RC = 4;     //RC PPM input
const int FREQ = 50;  //50Hz servo pwm frequency

const unsigned long SERVO1_MAX_US = 1700;
const unsigned long SERVO1_MIN_US = 1300;

const unsigned long SERVO2_MAX_US = 1800;
const unsigned long SERVO2_MIN_US = 1200;

const unsigned long WATCHDOG_DELAY = 5;

//RC Override
unsigned long usLastPulse;
unsigned long usPrevPulse;
unsigned long usDelta;
unsigned long pulseLengths[6];
int pulseCount = 0;

typedef struct{
  char cmd_rx[64];
  int thrust_sp;
  int rudder_sp;
  int servo1_us;
  int servo2_us;
  bool rc;
  int wd_count;
} StructState;

typedef struct {
  unsigned long *pPPM;
  StructState* pState;
} StructRC;

//Utility functions
void setServos(int us1, int us2) {
  analogWrite(SERVO1,long(1024 * us1 / 20000));
  analogWrite(SERVO2,long(1024 * us2 / 20000));
}