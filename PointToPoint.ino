

void WayPoint(float Xtraget , float Ytraget,float Heading , float DistanceP , float DistanceI ,float DistanceD) {
  static float ThetaTraget = 0, DistanceR = 0;
  static float ErrorDistance = 0 , PrevErrorDistance = 0 , SumErrorDistance = 0 , DErrorDistance = 0;
  static double DxTraget = 0, DyTraget = 0;
  static float CommandSpeed , CommandHeading;
  static float Xr = 0, Yr = 0 ;




  while (true) {
    imu_run();
    Distance();
    DxTraget = Xtraget - SxCompensate;
    DyTraget = Ytraget - SyCompensate;
    //    Serial.print(SxCompensate);
    //    Serial.print(" , ");
    //    Serial.println(SyCompensate);
    ThetaTraget = atan2( DyTraget, DxTraget );
    ThetaTraget = ThetaTraget * RAD_TO_DEG;
    //    Serial.print(ThetaTraget );
    //    Serial.print(" , ");
    Xr = SxCompensate - Xtraget;
    Yr = SyCompensate - Ytraget;
    DistanceR = sqrt( pow(Xr , 2) + pow(Yr , 2) );
    //    Serial.print(DxTraget);
    //    Serial.print(" , ");
    //    Serial.print(DyTraget);
    //    Serial.print(" , ");

    ///////////////PID Distance////////////////

    ErrorDistance = (DistanceR) ;
    //    Serial.println(ErrorDistance);
    SumErrorDistance += ErrorDistance;
    SumErrorDistance = constrain(SumErrorDistance, -100000, 100000);

    DErrorDistance = ErrorDistance - PrevErrorDistance;
    PrevErrorDistance = ErrorDistance;

    CommandSpeed = (DistanceP * ErrorDistance) + (DistanceI * SumErrorDistance) + (DistanceD * DErrorDistance);
    CommandHeading = ControlHeading(ypr_readable[0] , Heading , 0.72 , 0.1 , 0 );



    //////////////////////////////////////////
    if (ErrorDistance >= 0.02) {

      motorSpeedA = _Move1(abs(CommandSpeed) + 10 , ThetaTraget , CommandHeading);
      motorSpeedB = _Move2(abs(CommandSpeed) + 10 , ThetaTraget , CommandHeading);
      motorSpeedC = _Move3(abs(CommandSpeed) + 10 , ThetaTraget , CommandHeading);
      motorSpeedD = _Move4(abs(CommandSpeed) + 10 , ThetaTraget , CommandHeading);
      speedControl(flWheel , motorSpeedA); //FL
      speedControl(frWheel , motorSpeedB); //FR
      speedControl(brWheel , motorSpeedC); //BR
      speedControl(blWheel , motorSpeedD); //BL

    }
    else {
      speedControl(flWheel , 0); //FL
      speedControl(frWheel , 0); //FR
      speedControl(brWheel , 0); //BR
      speedControl(blWheel , 0); //BL
      
      break;

    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double VelocityControl(float Setpoint,float FeedBack,float kp,float ki,float kd){
  static float ErrorV = 0;
  static float PrevErrorV = 0;
  static float DErrorV = 0;
  static double SumErrorV = 0;
  static float  Command = 0;
  
  ErrorV = Setpoint - FeedBack;
  SumErrorV += ErrorV;
  SumErrorV = constrain(SumErrorV, -10000, 10000); 
  DErrorV = ErrorV - PrevErrorV;
  PrevErrorV = ErrorV;
  
  Command = (kp * ErrorV) + (ki * SumErrorV) + (kd * DErrorV);
  return Command;
  }
