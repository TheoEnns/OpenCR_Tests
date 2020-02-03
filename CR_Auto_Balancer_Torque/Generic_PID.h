#include "Arduino.h"

class PID
{
  public:

    //Constants used in some of the functions below
    #define DIRECT  0
    #define REVERSE  1
  
    //commonly used functions **************************************************************************
    PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd,
        int polarity, double _min, double _max);

    bool Compute(double deltaTime);
    void SetLimitations(double _min, double _max); 
    
    //available but not commonly used functions ********************************************************
    void SetTunings(double Kp, double Ki, double Kd);     
  
    void SetPolarity(int _polarity);        
                        
    //Display functions 
    double GetKp();              
    double GetKi();              
    double GetKd();              
    int GetPolarity();           

  private:
    void init();       

    double kp;           
    double ki;                  
    double kd;                  
    
    int polarity;
  
    double *myInput;              
    double *refOutput;             
    double *mySetpoint;      
    
    double outputIntegral, lastInput;
    double minOutput, maxOutput;
};

/*Constructor */
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd,
        int polarity, double _min, double _max)
{
    refOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;

    PID::SetLimitations(_min, _max); 

    PID::SetPolarity(polarity);
    PID::SetTunings(Kp, Ki, Kd);
}


/* Compute() */
bool PID::Compute(double deltaTime)
{
   double timeChange = deltaTime;

    /*Compute all the working error variables*/
    double input = *myInput;
    double error = *mySetpoint - input;
    double dInput = (input - lastInput)/deltaTime;
    outputIntegral+= (ki * error)*deltaTime;

    if(outputIntegral > maxOutput) 
      outputIntegral= maxOutput;
    else if(outputIntegral < minOutput) 
      outputIntegral= minOutput;

    double output;
    output = kp * error;

    /*Compute Rest of PID Output*/
    output += outputIntegral - kd * dInput;

    if(output > maxOutput) 
      output = maxOutput;
    else if (output < minOutput) 
      output = minOutput;
    *refOutput = output;

    /*Remember some variables for next time*/
    lastInput = input;
    return true;
}

/* SetTunings */
void PID::SetTunings(double Kp, double Ki, double Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;

  if(polarity == REVERSE)
  {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
}

/* SetLimitations */
void PID::SetLimitations(double _min, double _max)
{
   if(_min >= _max) 
     return;
   minOutput = _min;
   maxOutput = _max;

   if(*refOutput > maxOutput) 
      *refOutput = maxOutput;
   else 
      if(*refOutput < minOutput) 
        *refOutput = minOutput;

   if(outputIntegral > maxOutput) 
      outputIntegral= maxOutput;
   else 
      if(outputIntegral < minOutput) 
        outputIntegral= minOutput;
}

/* init() */
void PID::init()
{
   outputIntegral = *refOutput;
   lastInput = *myInput;
   if(outputIntegral > maxOutput) 
    outputIntegral = maxOutput;
   else if(outputIntegral < minOutput) 
    outputIntegral = minOutput;
}

/* SetPolarity */
void PID::SetPolarity(int _polarity)
{
   if( _polarity !=polarity)
   {
      kp = -kp;
      ki = -ki;
      kd = -kd;
   }
   polarity = _polarity;
}

/* Getter Accessors */
double PID::GetKp()
{
  return  kp; 
}

double PID::GetKi()
{
  return  ki;
}

double PID::GetKd()
{ 
  return  kd;
}

int PID::GetPolarity()
{ 
  return polarity;
}
