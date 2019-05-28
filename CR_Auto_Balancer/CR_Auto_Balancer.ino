#include <IMU.h>
#include <math.h>
#include <DynamixelWorkbench.h>
#include "pitches.h"
#include "Generic_PID.h"


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
#define RIGHT_MOTOR   1
#define LEFT_MOTOR    2
#define MAX_WHEEL_SPEED     300
uint8_t dxl_id[2] = {RIGHT_MOTOR, LEFT_MOTOR};
int32_t raw_position[2] = {0, 0};
int32_t goal_velocity[2] = {0, 0};
const uint8_t handler_index = 0;


/*******************************************************************************
* PID loop
*******************************************************************************/
double DistL_Kp = 5.00*0.01; 
double DistL_Ki = 0.00*0.01; 
double DistL_Kd = 0.00*0.01; 
double DistR_Kp = 5.00*0.01; 
double DistR_Ki = 0.00*0.01; 
double DistR_Kd = 0.00*0.01; 
double VelR_Kp = 20.000; 
double VelR_Ki = 0.00; 
double VelR_Kd = 0.00; 
double VelL_Kp = 20.000; 
double VelL_Ki = 0.00; 
double VelL_Kd = 0.00;

void setup() {
  Serial.begin(115200);
  const char *log;

  // Signal Start Operation
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BDPIN_BUZZER, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BDPIN_BUZZER);
  }

  delay(1000);

  IMU.begin(667); // 1500 us per cycle
                  // This controls the update rate of the whole control scheme
  //IMU.SEN.acc_cali_start();
    
  // Setting for Dynamixel motors
  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb.ping(RIGHT_MOTOR);
  dxl_wb.ping(LEFT_MOTOR);

  dxl_wb.wheelMode(RIGHT_MOTOR);
  dxl_wb.wheelMode(LEFT_MOTOR);

  bool result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Velocity", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }
  delay(250);
}

void loop() {
  const char *log;
  double right_wheel_vel = 0;
  double left_wheel_vel = 0;
  
  double target_dist_right=0, target_dist_left=0;
  double cur_dist_right=0, cur_dist_left=0;
  double angle, target_angle_right=0, target_angle_left=0;
  double target_vel_right=0, target_vel_left=0;

  PID pidAngleR(&cur_dist_right, &target_angle_right, &target_dist_right,
                DistL_Kp, DistL_Ki, DistL_Kd,
                DIRECT, -90, 90);
  PID pidAngleL(&cur_dist_left, &target_angle_left, &target_dist_left,
                DistR_Kp, DistR_Ki, DistR_Kd,
                DIRECT, -90, 90);
  PID pidVelR(&angle, &target_vel_right, &target_angle_right,
              VelR_Kp, VelR_Ki, VelR_Kd,
              DIRECT, -300, 300);
  PID pidVelL(&angle, &target_vel_left, &target_angle_left,
              VelL_Kp, VelL_Ki, VelL_Kd,
              DIRECT, -300, 300);
  
  double delta;
  uint32_t curTime, lastTime = micros();
  
  //do not use outer loop to avoid time delay
  int count=0;
  while( true ){
    //Maintian rate
    // Disabled since the IMU check on OpenCR IMU.update gates the refresh cycle already
    curTime = micros();
    delta = ((double)(curTime) - (double)(lastTime))*0.000001;
    if(delta < 0.0014){
      delayMicroseconds( (uint32_t)(1000000*(0.0015 - delta)));
    }
    
    // Get Time update
    curTime = micros();
    delta = ((double)(curTime) - (double)(lastTime))*0.000001;
    
    //IMU Update
    if(!(IMU.update() > 0)) {
      // Serial.println("IMU Read Failure"); // OpenCr fails to read the 
      continue;                             // IMU repeatedly before success
    }
    angle = IMU.rpy[1];

    //Read Present Position
    if( !(dxl_wb.itemRead(RIGHT_MOTOR, "Present_Position", &raw_position[0], &log) && 
          dxl_wb.itemRead(LEFT_MOTOR, "Present_Position", &raw_position[1], &log)) ) {
      Serial.println(log);
      Serial.println("getSyncReadData fail"); 
      continue;
    }
    cur_dist_right = raw_position[0] - right_wheel_vel*delta;
    cur_dist_left = raw_position[1] - left_wheel_vel*delta;

    //Update PIDs
    pidAngleR.Compute(delta);
    pidAngleL.Compute(delta);
    pidVelR.Compute(delta);
    pidVelL.Compute(delta);

    //Send Motor Update
    goal_velocity[0] = target_vel_right;
    goal_velocity[1] = target_vel_left;
    if(!dxl_wb.syncWrite(handler_index, &goal_velocity[0], &log)) {
      Serial.println(log);
      Serial.println("goalVelocity fail"); 
      continue;
    }

    // Store Last Time
    lastTime = curTime;

    //Send update
    if(count>200){
      Serial.print(raw_position[0]);
      Serial.print(" \t");
      Serial.print(raw_position[1]);
      Serial.print(" \t");
      Serial.print(target_angle_right);
      Serial.print(" \t");
      Serial.print(target_angle_left);
      Serial.print(" \t");
      Serial.print(goal_velocity[0]);
      Serial.print(" \t");
      Serial.print(goal_velocity[1]);
      Serial.print(" \t");
      Serial.print(angle);
      Serial.print(" \t");
      Serial.println(delta*1000000);
      count=0;
    }
    count++;
  }
}
