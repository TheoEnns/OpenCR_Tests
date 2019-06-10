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
// FYI: See the Ziegler–Nichols tuning method: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
//   or just do manual tuning: https://en.wikipedia.org/wiki/PID_controller#Manual_tuning
double DistL_Kp = 15.00*0.001; 
double DistL_Ki = 15.00*0.001; 
double DistL_Kd = 0.00*0.001; 
double DistR_Kp = 15.00*0.001; 
double DistR_Ki = 15.00*0.001; 
double DistR_Kd = 0.00*0.001; 
/*
double DistL_Kp = 1.00*0.01; 
double DistL_Ki = 20.00*0.01; 
double DistL_Kd = 0.00*0.0001; 
double DistR_Kp = 1.00*0.01; 
double DistR_Ki = 20.00*0.01; 
double DistR_Kd = 0.00*0.0001; 
 */
double VelR_Kp = 18.000; 
double VelR_Ki = 4.50; 
double VelR_Kd = 0.30; 
double VelL_Kp = 18.000; 
double VelL_Ki = 4.50; 
double VelL_Kd = 0.30;
int angle_adjust_ratio = 1;

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
                DistR_Kp, DistR_Ki, DistR_Kd,
                DIRECT, -20, 20);
  PID pidAngleL(&cur_dist_left, &target_angle_left, &target_dist_left,
                DistL_Kp, DistL_Ki, DistL_Kd,
                DIRECT, -20, 20);
  PID pidVelR(&angle, &target_vel_right, &target_angle_right,
              VelR_Kp, VelR_Ki, VelR_Kd,
              DIRECT, -300, 300);
  PID pidVelL(&angle, &target_vel_left, &target_angle_left,
              VelL_Kp, VelL_Ki, VelL_Kd,
              DIRECT, -300, 300);
  
  double delta;
  uint32_t curTime, lastTime = micros();
  
  //do not use outer loop to avoid time delay
  int text_count=0;
  int angle_count=0;
  while( true ){
    // IMU check on OpenCR IMU.update gateways the refresh cycle already    
    // Get Time update
    curTime = micros();
    delta = ((double)(curTime) - (double)(lastTime))*0.000001;
    if(delta < 0)
    {
      delta = 0.0015; //Rollover Protection; takes 49ish days to happen though
    }
    
    //IMU Update
    if(!(IMU.update() > 0)) {
      // Serial.println("IMU Read Failure"); // OpenCr fails to read the 
      continue;                             // IMU repeatedly before success
    }
    angle = IMU.rpy[1];
    curTime = micros();
    delta = ((double)(curTime) - (double)(lastTime))*0.000001;
    

    //Read Present Position
    cur_dist_right = (target_vel_right - right_wheel_vel);
    cur_dist_left = (target_vel_right - left_wheel_vel);
    if(angle_count >= angle_adjust_ratio){
      angle_count = 0;
      
      //Update PIDs
      pidAngleR.Compute(delta);
      pidAngleL.Compute(delta);
    } else {
      angle_count++;
    }

    //Update PIDs
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
    if(text_count > 200){
      Serial.print(cur_dist_right);
      Serial.print(" \t");
      Serial.print(cur_dist_left);
      Serial.print(" \t");
      Serial.print(target_dist_right);
      Serial.print(" \t");
      Serial.print(target_dist_left);
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
      Serial.print(delta*1000000);
      Serial.print(" \t");
      Serial.print(((double)(micros()) - (double)(curTime)));
      Serial.println("");
      text_count=0;
    } else {
      text_count ++;
    }
  }
}
