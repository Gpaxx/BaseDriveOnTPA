float Sx = 0, Sy = 0;
float PrevSx = 0 , PrevSy = 0;

float dPluseFC, dPluseBL, dPluseBR;
float prevPluseFC = 0, prevPluseBL = 0, prevPluseBR = 0;


void readPluse() {
  ENC1 = fcEncoder.read();
  ENC2 = blEncoder.read();
  ENC3 = brEncoder.read();

  if (ENC1 != currentPluseFC || ENC2 != currentPluseBL || ENC3 != currentPluseBR ) {

    currentPluseFC = ENC1;
    currentPluseBL = ENC2;
    currentPluseBR = ENC3;

  }
}

void Distance() {
  
   readPluse();

  // constant r = 0.029
  Sx = 0.00052902054 * ((( currentPluseBL * (0.5) * 0.029  )  +  (currentPluseBR * (0.5) * 0.029 )) + (-currentPluseFC * 0.029)); //00153318032
  Sy = 0.00052902054 *  ( (currentPluseBR * (0.8660254) * 0.029) +  (currentPluseBL * (-0.8660254) * 0.029) );//

  dx = Sx - PrevSx;
  dy = Sy - PrevSy;
  if (dx != 0.000000000000000000000 || dy != 0.000000000000000000000){
    Serial.println(String(dy)+String(Sy));
  }
  PrevSx = Sx;
  PrevSy = Sy;
  

  SxCompensate = SxCompensate + (cos(-ypr[0] ) * dx) - (sin(-ypr[0]) * dy);
  SyCompensate = SyCompensate + (sin(-ypr[0]) * dx) + (cos(-ypr[0]) * dy);

  //  Serial.println(ypr[0]/);
  //  Serial.println("Sx : " + String(Sx) + " , Sy : " + String(Sy) + " //// " + String("xC: ") + String(sx_compensate) + "," + String("yC: ") + String(sy_compensate)  +String("yaw: ") + String(ypr_readable[0]));
  //  Serial.println("Sx : " + String(Sx) + " , Sy : " + String(Sy) + " //// " + String("xC: ") + String(SxCompensate) + "," + String("yC: ") + String(SyCompensate));
  //  Serial.println("current/PluseFC : " + String(currentPluseFC) + "  currentPl/useBL: " + String(currentPluseBL) + "  currentPluseBR: " + String(currentPluseBR));

}

//double FeedBack(double DeltaX , double DeltaY ,double Omega){
//    
//  }

void ResetDistance(){
  ENC1 = 0;
  ENC2 = 0;
  ENC3 = 0;
  currentPluseFC = 0;
  currentPluseBL = 0;
  currentPluseBR = 0 ;
  PrevSx = 0;
  PrevSy = 0;
  
  }
