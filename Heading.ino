double ControlHeading (int Value , int SetPoint , float kp , float ki, float kd) {
  static float Error = 0 , PrevError = 0 , SumError = 0 , DError = 0;
  static float P = 0 ,I = 0 ,D = 0;
  static float PID;

  if (Value < 180 && Value >= 0){
    ThetaRobot = ypr_readable[0];
      }
  else if (Value >= -180 && Value < 0 ){
    ThetaRobot = map(ypr_readable[0] ,-180, 0 , 180 , 360 );
      }

//  Error = 90 - 180  
  Error = SetPoint - ThetaRobot;
  if (Error < -180 ){
    Error += 360;
    }
  if (Error > 180 ){
    Error -= 360;
    }
  
  P = kp * Error;
  
  SumError += Error;
  SumError = constrain(SumError, -10000, 10000);
  I = ki * SumError; 

  
  DError = Error - PrevError;
  PrevError = Error; 
  D = kd * DError;
  
  PID = P + I + D; 
  
  

  
  return PID;//void controller_pid() {
}

void Heading(int SetPoint ){
  CommandHeading =  ControlHeading(ypr_readable[0] , SetPoint , 0.72 , 0.1 , 0);
  speedControl(flWheel , _Move1(0 , 0 , CommandHeading ) ); //FL
  speedControl(frWheel , _Move2(0 , 0 , CommandHeading ) ); //FR
  speedControl(brWheel , _Move3(0 , 0 , CommandHeading ) ); //BR
  speedControl(blWheel , _Move4(0 , 0 , CommandHeading ) ); //BL
  }
