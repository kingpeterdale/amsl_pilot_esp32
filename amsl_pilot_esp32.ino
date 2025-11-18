#include "pilot_esp32.h"
#include "pilot_tasks.h"

// Global State 
StructState state;
StructRC rc = {pulseLengths, &state};

void setup() {
  pinMode(SERVO1,OUTPUT);
  pinMode(SERVO2,OUTPUT);
  pinMode(RC, INPUT);
  Serial.begin(9600);

  // Setup ESC
  analogWrite(SERVO1,long(1024 * 1500 / 20000));
  analogWrite(SERVO2,long(1024 * 1500 / 20000));
  vTaskDelay(pdMS_TO_TICKS(3000));

  //RC Override
  state.wd_count = 0;
  state.rc = false;
  attachInterrupt(digitalPinToInterrupt(RC), rcInterrupt, RISING);

  state.servo1_us = 1500;
  state.servo2_us = 1500;
  analogWriteResolution(10);
  analogWriteFrequency(50);

  xCommandSemaphore = xSemaphoreCreateBinary();
  xMotorSemaphore = xSemaphoreCreateBinary();
  xTaskCreate(TaskRCOverride, "TaskRCOverride",4096,(void *)&rc,3,&taskRC);
  xTaskCreate(TaskWatchdog, "TaskWatchdog",4096,(void *)&state,3,&taskWatchdog);
  xTaskCreate(TaskParse,"TaskParse",4096,(void *)&state,2,&taskParse);
  xTaskCreate(TaskStatus,"TaskStatus",4096,(void *)&state,1,&taskStatus);
  xTaskCreate(TaskReadSerial, "TaskReadSerial",4096,(void *)&state,2,&taskReadSerial);
  xTaskCreate(TaskMotor, "TaskMotor",4096,(void *)&state,2,&taskMotor);

}

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

void loop() {
}
