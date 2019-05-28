#include <IMU.h>
#include <DynamixelWorkbench.h>
#include "pitches.h"

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

/*******************************************************************************
* Audio Generation
*******************************************************************************/
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};


/*******************************************************************************
* IMU
*******************************************************************************/
cIMU    IMU;

/*******************************************************************************
* Motor Driver
*******************************************************************************/
DynamixelWorkbench dxl_wb;
#define BAUDRATE  3000000
#define RIGHT_MOTOR_A   1
#define LEFT_MOTOR_A    2
uint8_t dxl_id[2] = {LEFT_MOTOR_A, RIGHT_MOTOR_A};
int32_t goal_position[2] = {20, 20};
int32_t present_position[2] = {0, 0};
const uint8_t handler_index = 0;

void setup() {
  Serial.begin(115200);
  const char *log;

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BDPIN_BUZZER, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BDPIN_BUZZER);
  }

  delay(1000);
    
  // Setting for Dynamixel motors
  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb.ping(RIGHT_MOTOR_A);
  dxl_wb.ping(LEFT_MOTOR_A);

  dxl_wb.wheelMode(RIGHT_MOTOR_A);
  dxl_wb.wheelMode(LEFT_MOTOR_A);

  bool result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Velocity", &log);
  if (result == false)
  {
    delay(1000);
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }

  result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Position", &log);
  if (result == false)
  {
    delay(1000);
    Serial.println(log);
    Serial.println("Failed to add sync read handler");
  }

  IMU.begin();
}

void loop()
{
  const char *log;
  static int count = 0;
  static uint32_t tTime[4];
  static float averages[10];
  static uint32_t imu_time = 0;
  int k = 0;

  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();
    goal_position[0] = (goal_position[0] + 3)%100;
    goal_position[1] = (goal_position[1] + 3)%100;
    count = 0;
  }
  count ++;

  tTime[2] = micros();
  if( IMU.update() > 0 ) {
    imu_time = micros()-tTime[2];
    averages[k] = (averages[k]*99.0 + imu_time)/100.0; k++;
  } else {
//    Serial.println("IMU fail"); 
    k++;
  }


  tTime[2] = micros(); //dxl_id

  //Fails Second Read
//  if( dxl_wb.syncRead(handler_index, &log) && dxl_wb.getSyncReadData(handler_index, &present_position[0], &log)) {

  //Works at ~815 us
//  if( dxl_wb.syncRead(handler_index, &dxl_id[0], 1, &log) && dxl_wb.getSyncReadData(handler_index, &dxl_id[0], 1, &present_position[0], &log) && 
//      dxl_wb.syncRead(handler_index, &dxl_id[1], 1, &log) && dxl_wb.getSyncReadData(handler_index, &dxl_id[1], 1, &present_position[1], &log)) {

  //Fails Second Read
//  if( dxl_wb.syncRead(handler_index, dxl_id, 2, &log) && dxl_wb.getSyncReadData(handler_index, dxl_id, 2, &present_position[0], &log)) {

//  //Works at 800 us
  if( dxl_wb.itemRead(LEFT_MOTOR_A, "Present_Position", &present_position[0], &log) && dxl_wb.itemRead(RIGHT_MOTOR_A, "Present_Position", &present_position[1], &log)) {
    imu_time = micros()-tTime[2];
    averages[k] = (averages[k]*99.0 + imu_time)/100.0; k++;
  } else {
//    Serial.println(log);
//    Serial.println("getSyncReadData fail"); 
    k++;
  }
  
  tTime[2] = micros();
//  if( dxl_wb.goalVelocity(RIGHT_MOTOR_A, (int32_t)0)) {
  if( dxl_wb.syncWrite(handler_index, &goal_position[0], &log)) {
    imu_time = micros()-tTime[2];
    averages[k] = (averages[k]*99.0 + imu_time)/100.0; 
    k++;
  } else {
    Serial.println(log);
    Serial.println("goalVelocity fail"); 
    k++;
  }
  
  averages[k] = (averages[k]*99.0 + IMU.rpy[0])/100.0; k++;
  averages[k] = (averages[k]*99.0 + IMU.rpy[1])/100.0; k++;
  averages[k] = (averages[k]*99.0 + IMU.rpy[2])/100.0; k++;


  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();
    for(int i=0;i<k;i++){
      Serial.print(averages[i]);
      Serial.print(" \t");
    }
    Serial.print(present_position[0]);
    Serial.print(" \t");
    Serial.print(present_position[1]);
    Serial.print(" \t");
    Serial.print(count);
    Serial.println("");
  }


  if( Serial.available() )
  {
    char Ch = Serial.read();

    if( Ch == '1' )
    {
      Serial.println("ACC Cali Start");

      IMU.SEN.acc_cali_start();
      while( IMU.SEN.acc_cali_get_done() == false )
      {
        IMU.update();
      }

      Serial.print("ACC Cali End ");
    }
  }
}
