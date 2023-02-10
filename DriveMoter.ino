int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeedC = 0;
int motorSpeedD = 0;
float CommandHeading;

int Vx = 0,Vy = 0;

void DriveMotor(int Mode) {

  if (Mode == 0 ) {
    CtrlDirectionJoy();
    calOmega();
    bool state = stateJoy;
    Distance();
    if (joy.isConnected() == 1) {
      if (state == true) {
        motorSpeedA = _Move1(50 , alphaDegree , omega);
        motorSpeedB = _Move2(50 , alphaDegree , omega);
        motorSpeedC = _Move3(50 , alphaDegree , omega);
        motorSpeedD = _Move4(50 , alphaDegree , omega);

        Serial1.flush();
        speedControl(flWheel , motorSpeedA); //FL
        speedControl(frWheel , motorSpeedB); //FR
        speedControl(brWheel , motorSpeedC); //BR
        speedControl(blWheel , motorSpeedD); //BL
      }
      else if (state == false && abs(omega) > 9) {
        //        /Serial.println(omega);
        motorSpeedA = _Move1(0 , 0 , omega);
        motorSpeedB = _Move2(0 , 0 , omega);
        motorSpeedC = _Move3(0 , 0 , omega);
        motorSpeedD = _Move4(0 , 0 , omega);
        //        Serial.println(motorSpeedA);
        speedControl(flWheel , motorSpeedA); //FL
        speedControl(frWheel , motorSpeedB); //FR
        speedControl(brWheel , motorSpeedC); //BR
        speedControl(blWheel , motorSpeedD); //BL
      }

      else {
        //        Serial.println("mana ");
        speedControl(flWheel , 0); //FL
        speedControl(frWheel , 0); //FR
        speedControl(brWheel , 0); //BR
        speedControl(blWheel , 0); //BL

        state = false;
      }
    }
    else {

      //stop
      speedControl(flWheel , 0); //FL
      speedControl(frWheel , 0); //FR
      speedControl(brWheel , 0); //BR
      speedControl(blWheel , 0); //BL
    }

  }
  else if (Mode == 1) {
    delay(500);
    WayPoint( 0 ,2 , 30.0 , 0.0001 , 0 ,0 );
    speedControl(flWheel , 0); //FL
    speedControl(frWheel , 0); //FR
    speedControl(brWheel , 0); //BR
    speedControl(blWheel , 0); //BL
    delay(1000);

    WayPoint( -1,2 , 30.0 , 0.0001 , 0 ,0 );
    speedControl(flWheel , 0); //FL
    speedControl(frWheel , 0); //FR
    speedControl(brWheel , 0); //BR
    speedControl(blWheel , 0); //BL
    delay(1000);
    WayPoint(-1 ,3 , 30.0 , 0.0001 , 0,0  );
    speedControl(flWheel , 0); //FL
    speedControl(frWheel , 0); //FR
    speedControl(brWheel , 0); //BR
    speedControl(blWheel , 0); //BL
    delay(1000);
//    ControlHeading()
    
   
//    Serial.println("stop");
  }
  else if(Mode == 2){
    Distance();
//    Vx = VelocityControl(5 ,dx ,0,0,0);//Setpoint, FeedBack, kp, ki ,  kd
//    Vy = VelocityControl(5 ,dy ,0,0,0);//Setpoint, FeedBack, kp, ki ,  kd
//    CommandHeading = ControlHeading(90,ypr_readable[0],0,0,0) ;
    motorSpeedA =  _ControlV1(0 , 0.5, CommandHeading);
    motorSpeedB =  _ControlV2(0 , 0.5 , CommandHeading);
    motorSpeedC =  _ControlV3(0 , 0.5 , CommandHeading);
    motorSpeedD =  _ControlV4(0 , 0.5 , CommandHeading);

//    Serial.println(String(motorSpeedA) + " " + String(motorSpeedB) + " " + String(motorSpeedC) + " " + String(motorSpeedD));
    //Serial.println(String(dx) + " " + String(dy));
    
    speedControl(flWheel , motorSpeedA); //FL
    speedControl(frWheel , motorSpeedB); //FR
    speedControl(brWheel , motorSpeedC); //BR
    speedControl(blWheel , motorSpeedD); //BL
    }
  else {
    //stop
    speedControl(flWheel , 0); //FL
    speedControl(frWheel , 0); //FR
    speedControl(brWheel , 0); //BR
    speedControl(blWheel , 0); //BL

  }

}

//////////////////////////////////////////////////////////////////////Direction to traget Command:Alpha/////////////////////////////////////////////////////////////////////////////////////

int _Move1(int rpm , int alpha , int omega) {
  return (rpm * (((cos((-135 + (ypr_readable[0])) * DEG_TO_RAD )) * cos(alpha * DEG_TO_RAD)) + (sin((-135 + (ypr_readable[0])) * DEG_TO_RAD )) * sin(alpha * DEG_TO_RAD))) + omega;
}

int _Move2(int rpm , int alpha , int omega) {
  return (rpm * (((cos(( 135 + (ypr_readable[0])) * DEG_TO_RAD )) * cos(alpha * DEG_TO_RAD)) + (sin(( 135 + (ypr_readable[0])) * DEG_TO_RAD )) * sin(alpha * DEG_TO_RAD))) + omega;
}

int _Move3(int rpm , int alpha , int omega) {
  return (rpm * (((cos((  45 + (ypr_readable[0])) * DEG_TO_RAD )) * cos(alpha * DEG_TO_RAD)) + (sin((  45 + (ypr_readable[0])) * DEG_TO_RAD )) * sin(alpha * DEG_TO_RAD))) + omega;
}

int _Move4(int rpm , int alpha , int omega) {
  return (rpm * (((cos(( -45 + (ypr_readable[0])) * DEG_TO_RAD )) * cos(alpha * DEG_TO_RAD)) + (sin(( -45 + (ypr_readable[0])) * DEG_TO_RAD )) * sin(alpha * DEG_TO_RAD))) + omega;
}


//////////////////////////////////////////////////////////////////////Control speed Command:Vx,Vy,Heading/////////////////////////////////////////////////////////////////////////////////////
//r = 0.029 m #wheel 1/r = 34.4827586


double _ControlV1(float Vx,float Vy,int Heading) {
  return ((1/0.029)*((cos((-135 + (ypr_readable[0])) * DEG_TO_RAD )*Vx) + (sin((-135 + (ypr_readable[0])) * DEG_TO_RAD )*Vy) + (omega)));
  }
double _ControlV2(float Vx,float Vy,int Heading) {
  return ((1/0.029)*((cos(( 135 + (ypr_readable[0])) * DEG_TO_RAD )*Vx) + (sin(( 135 + (ypr_readable[0])) * DEG_TO_RAD )*Vy) + (omega)));
  }
double _ControlV3(float Vx,float Vy,int Heading) {
  return ((1/0.029)*((cos((  45 + (ypr_readable[0])) * DEG_TO_RAD )*Vx) + (sin((  45 + (ypr_readable[0])) * DEG_TO_RAD )*Vy) + (omega)));
  }
double _ControlV4(float Vx,float Vy,int Heading) {
  return ((1/0.029)*((cos(( -45 + (ypr_readable[0])) * DEG_TO_RAD )*Vx) + (sin(( -45 + (ypr_readable[0])) * DEG_TO_RAD )*Vy) + (omega)));
  }
