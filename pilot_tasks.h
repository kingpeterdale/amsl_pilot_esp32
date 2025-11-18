

SemaphoreHandle_t xCommandSemaphore;
SemaphoreHandle_t xMotorSemaphore;

TaskHandle_t taskParse;
TaskHandle_t taskStatus;
TaskHandle_t taskReadSerial;
TaskHandle_t taskMotor;
TaskHandle_t taskRC;
TaskHandle_t taskWatchdog;

void TaskRCOverride(void *pvParameters) {
  /**
    Monitors RC input and enacts override if requested
  */
  StructRC *pRC = (StructRC *)pvParameters;
  while(true) {
    if (pRC->pPPM[4] == 2000) {
      pRC->pState->wd_count = 0;
      pRC->pState->rc = true;
      pRC->pState->servo1_us = min(pRC->pPPM[1],SERVO1_MAX_US);
      pRC->pState->servo2_us = min(pRC->pPPM[0],SERVO2_MAX_US);
      setServos(pRC->pState->servo1_us,pRC->pState->servo2_us);
    }
    else pRC->pState->rc = false;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskWatchdog(void *pvParameters) {
  StructState *state = (StructState *)pvParameters;
  while (true){
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (state->wd_count >= WATCHDOG_DELAY) {
      setServos(1500,1500);
      state->servo1_us = 1500;
      state->servo2_us = 1500;
    }
    state->wd_count++;
  }
}

/**
  Awaits a semaphore that a new command has been received.
  Parses the command and updates the setpoints
*/
void TaskParse(void *pvParameters) {
  StructState *state = (StructState *)pvParameters;
  while(true) {
    if (xSemaphoreTake(xCommandSemaphore,portMAX_DELAY) == pdPASS){
      state->wd_count = 0;
      //Serial.print("Parsing ");
      //Serial.println(state->cmd_rx);
      int count = sscanf(state->cmd_rx, "$CMD,%i,%i",&state->thrust_sp, &state->rudder_sp);
      if (count<2) {
        state->thrust_sp = 0;
        state->rudder_sp = 0;
      }
      xSemaphoreGive(xMotorSemaphore);
    }
  }
}

/**
  Sends out state every 1 second
*/
void TaskStatus(void *pvParameters){
  StructState *state = (StructState *)pvParameters;
  char status[64];
  int count = 0;
  while(true) {
    sprintf(status,"$STS,%03i,%+04i,%+04i,%04i,%04i",count,state->thrust_sp,state->rudder_sp,state->servo1_us,state->servo2_us);
    Serial.println(status);
    Serial.println(state->wd_count);
    count = (count+1)%1000;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
  Receives data on the serial port and when a newline is read, 
  informs that a command has been received
*/
void TaskReadSerial(void *pvParameters){
  StructState *state = (StructState *)pvParameters;
  String msg = "";
  while(true){
    if (Serial.available()){
      char inChar = (char)Serial.read();
      if (inChar == '\n'){
        //Serial.println("RXD " + msg);
        msg.toCharArray(state->cmd_rx,sizeof(state->cmd_rx));
        xSemaphoreGive(xCommandSemaphore);
        msg = "";
      }else {
        msg += inChar;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void TaskMotor(void *pvParameters){
  StructState *state = (StructState *)pvParameters;
  int servo1;
  int servo2;
  while(true) {
    if (xSemaphoreTake(xMotorSemaphore,portMAX_DELAY) == pdPASS && state->rc == false){
      //Serial.println("Setting motors");
      servo1 = 1*state->thrust_sp + 0*state->rudder_sp + 0;
      servo2 = 0*state->thrust_sp + 1*state->rudder_sp + 0;
      
      state->servo1_us = map(servo1,0,100,SERVO1_MIN_US,SERVO1_MAX_US);;
      state->servo2_us = map(servo2,-100,100,SERVO2_MIN_US,SERVO2_MAX_US);;
      setServos(state->servo1_us,state->servo2_us);
    }
  }
}
