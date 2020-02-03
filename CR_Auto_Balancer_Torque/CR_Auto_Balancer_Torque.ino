#include <IMU.h>
#include <math.h>
#include <DynamixelWorkbench.h>
#include "pitches.h"
#include "Generic_PID.h"

/*
 * Balancing Model:
 *   Encoders <PID--> Translation, Orientation
 *   Translation - V*T <PID--> TargetAngle
 *   TargetAngle - Angle <PID--> balanceVel
 *   balanceVel + Orientation <-> MotorVel_R
 *   balanceVel - Orientation <-> MotorVel_L
 *   MotorVel_R <On Motor PID--> PWM_R
 *   MotorVel_L <On Motor PID--> PWM_L
 */

#define DEBUG  true

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
#define MAX_SAFE_ANGLE     20.0

/*******************************************************************************
* Motor Driver
*******************************************************************************/
DynamixelWorkbench dxl_wb;
#define BAUDRATE  3000000
#define RIGHT_MOTOR   1
#define LEFT_MOTOR    2
#define MAX_WHEEL_SPEED     880
uint8_t dxl_id[2] = {RIGHT_MOTOR, LEFT_MOTOR};
int32_t last_position[2] = {0, 0};
int32_t raw_position[2] = {0, 0};
int32_t goal_velocity[2] = {0, 0};
const uint8_t handler_index = 0;


/*******************************************************************************
* PID loop
*******************************************************************************/
// FYI: See the Ziegler–Nichols tuning method: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
//   or just do manual tuning: https://en.wikipedia.org/wiki/PID_controller#Manual_tuning
double imuAngle = 0;
double imuAngleLimited = 0;
double targetTVel = 0;
double targetRVel = 0;
double actualTrans = 0;
double actualTransLimited = 0;
double targetTrans = 0;
double targetAngle = 0;
double actualRot = 0;
double targetRot = 0;
double balanceVel = 0;
double robotVel = 0;
double motorVel_R = 0;
double motorVel_L = 0;
PID pidTrans2Angle(&actualTrans, &targetAngle, &targetTrans,
//              0.0, 0.0, 0.0,
              0.0001, 0.0, 0.0,  // Bad Strategy to use this PID
              DIRECT, -2, 2);
PID pidTrans2Vel(&actualTransLimited, &robotVel, &targetTrans,
              0.0, 0.0, 0.0,
//              100.0, 0.0, 0.5,
              REVERSE, -MAX_WHEEL_SPEED*0.5, MAX_WHEEL_SPEED*0.5);
PID pidAngle2Vel(&imuAngleLimited, &balanceVel, &targetAngle,
//              0.0, 0.0, 0.0,
              300.000, 5.000, 0.500,
              REVERSE, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

/*******************************************************************************
* Timers
*******************************************************************************/
uint32_t t_now;
uint32_t t_last_angle;
//uint32_t t_last_encoder;
uint32_t t_last_serial;
double t_delta;


/*******************************************************************************
* Body State
*******************************************************************************/
#define ENCODER_2_DIST 0.19091924886 
    // Multiply encoder ticks to get distance in mm
    // 124.46 * 2*pi / 4096
#define ENCODER_2_PI 1727.784027
    // Multiply encoder ticks to get distance in mm
    // ((105*(pi))/782.005243332)*4096

double getDeltaT( uint32_t t_now, uint32_t t_last){
  return ((double)(t_now) - (double)(t_last))*0.000001;
}

float updatesPerSecond = 500;
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

  IMU.begin(updatesPerSecond); // X us per cycle
                  // This controls the update rate of the whole control scheme
  //IMU.SEN.acc_cali_start();
    
  // Setting for Dynamixel motors
  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb.ping(RIGHT_MOTOR);
  dxl_wb.ping(LEFT_MOTOR);

//  dxl_wb.setPWMControlMode(RIGHT_MOTOR);
//  dxl_wb.setPWMControlMode(LEFT_MOTOR);
  dxl_wb.itemWrite(RIGHT_MOTOR, "Operating_Mode", 16, &log);
  dxl_wb.itemWrite(LEFT_MOTOR, "Operating_Mode", 16, &log);

  bool result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_PWM", &log);
  if (DEBUG && result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }
  dxl_wb.torque(RIGHT_MOTOR, 1, &log);
  dxl_wb.torque(LEFT_MOTOR, 1, &log);
}

double signum(double val) {
    return (0 < val) - (val < 0);
}

void loop() {
  const char *log;
  float periodicity = 0.004;

  //Delay to let IMU stablize
  t_now = micros();
  while(micros()- t_now < 250000){
    IMU.update();
  }

  IMU.update();
  t_last_angle = micros();
  
  while( true ){    
    
    // IMU Update
    if(!(IMU.update() > 0)) {   // OpenCr fails to read the 
      continue;                 // IMU repeatedly before success
    }
    imuAngle = IMU.rpy[1]; // (IMU.rpy[1]*1.0 + 1.0)/2.0;
    t_now = micros();
    double delta = getDeltaT( t_now, t_last_angle);
    if(delta < periodicity)
      continue;
    t_last_angle = t_now;

    // Encoder Update
    if( !(dxl_wb.itemRead(RIGHT_MOTOR, "Present_Position", &raw_position[0], &log) && 
            dxl_wb.itemRead(LEFT_MOTOR, "Present_Position", &raw_position[1], &log)) ) {
        DEBUG? Serial.println(log) : 0;
        DEBUG? Serial.println("getSyncReadData fail") : 0; 
        continue;
    }
    double velocityR = ((double)(raw_position[0] - last_position[0]))/delta;
    double velocityL = ((double)(raw_position[1] - last_position[1]))/delta;
    last_position[0] = raw_position[0];
    last_position[1] = raw_position[1];
    actualTrans += 0.5 * delta * (velocityR + velocityL) * ENCODER_2_DIST;
    actualRot += delta * (velocityR - velocityL) * ENCODER_2_PI;   

    // Update pidTrans2Angle    
//    if(fabs(targetTrans - actualTrans) < 20.0) // Eliminate Small Trans Error
//      actualTransLimited = targetTrans;  
//    else
       actualTransLimited = actualTrans*actualTrans*signum(actualTrans);
    pidTrans2Angle.Compute(delta); // Disabled as serial PIDs are worse than parrellel
   
    // Update pidAngle2Veland pidTrans2Vel  
//    if(fabs(targetAngle - imuAngle) < 0.1) // Eliminate Small Angle Error
//      imuAngleLimited = targetAngle;
//    else
      imuAngleLimited = imuAngle;
    pidTrans2Vel.Compute(delta);
    pidAngle2Vel.Compute(delta);

    // Update Individual Motor Vel
    double rotation = constrain(actualRot/(2*M_PI*300), -100,100);
    motorVel_R = constrain(balanceVel + robotVel - 10*rotation, -MAX_WHEEL_SPEED,MAX_WHEEL_SPEED);
    motorVel_L = constrain(balanceVel + robotVel + 10*rotation, -MAX_WHEEL_SPEED,MAX_WHEEL_SPEED);
    

    // Nihilate Aggregation on Stability
//    if(fabs(targetTrans - actualTrans) < 3.0 &&
//       fabs(targetRot - actualRot) < 0.05 &&
//       fabs(motorVel_R) < 3.0 &&
//       fabs(motorVel_L) < 3.0
//       ) {
//      actualTrans = 0;
//      actualRot = 0;
//    }

    //Send Motor Update
    goal_velocity[0] = motorVel_R;
    goal_velocity[1] = motorVel_L;
    if(!dxl_wb.syncWrite(handler_index, &goal_velocity[0], &log)) {
      DEBUG? Serial.println(log) : 0;
      DEBUG? Serial.println("goalVelocity fail") : 0; 
      continue;
    }
    
    //Send update
    if(t_last_serial - t_now > 200){
      Serial.print(((double)(micros()) - (double)(t_now)));
      Serial.print(" \t");
      Serial.print(delta*1000000);
      Serial.print(" \t[");
      Serial.print(imuAngle);
      Serial.print(" \t");
      Serial.print(targetAngle);
      Serial.print("] \t[");
//      Serial.print(targetTVel);
//      Serial.print(" \t");
//      Serial.print(targetRVel);
//      Serial.print(" \t");
      Serial.print(actualTrans);
      Serial.print(" \t");
      Serial.print(targetTrans);
      Serial.print("] \t[");
      Serial.print(actualRot);
      Serial.print(" \t");
      Serial.print(targetRot);
      Serial.print("] \t");
      Serial.print(balanceVel);
      Serial.print(" \t");
      Serial.print(motorVel_R);
      Serial.print(" \t");
      Serial.print(motorVel_L);
      Serial.println("");
      t_now = micros();
    }
  }
}
