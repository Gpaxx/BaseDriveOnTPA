float DeathZone(int _analogJoy) {
  if (_analogJoy <= 60 && _analogJoy >= -60) {
    _analogJoy = 0;

  }
  else {
    _analogJoy = _analogJoy;

  }
  return _analogJoy;
}

bool StateJoy(int _analogJoy) {
  if (_analogJoy <= 60 && _analogJoy >= -60) {
    stateAnalog = false;
  }
  else {
    stateAnalog = true;
  }
  return stateAnalog;
}


void calOmega() {
  //
  omega = map(joy.getAnalog(joy.PS4A_Rx) , 0 , 255 , 70 , -70);
  if (abs(omega) <= 10) {
    
    omega = 0;
    
  }
}



void CtrlDirectionJoy() {

  NewLx = map(joy.getAnalog(joy.PS4A_Lx), 0, 255, -127, 127 );
  NewLy = map(joy.getAnalog(joy.PS4A_Ly), 0, 255, 127, -127 );
  
  Lx = DeathZone(NewLx);
  stateAnalogx =  StateJoy(NewLx);
  Ly = DeathZone(NewLy);
  stateAnalogy =  StateJoy(NewLy);

  if (stateAnalogx == false && stateAnalogy == false) {
    alpha = atan2(Ly, Lx);
    alphaDegree = (alpha * RAD_TO_DEG);
    stateJoy = false;
  }
  else {
    alpha = atan2(Ly, Lx);
    alphaDegree = (alpha * RAD_TO_DEG);
    stateJoy = true;
  }
}
