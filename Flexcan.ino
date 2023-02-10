union int32_t2Uin8_t {
  int32_t asInt32_t = 0;
  uint8_t asUint8_t[4];
} M1, cmbACC;

void motorStop(int16_t _id) {
  CAN_message_t cmd;

  cmd.id = _id;
  cmd.buf[0] = 0x80;
  cmd.buf[1] = 0x00;
  cmd.buf[2] = 0x00;
  cmd.buf[3] = 0x00;
  cmd.buf[4] = 0x00;
  cmd.buf[5] = 0x00;
  cmd.buf[6] = 0x00;
  cmd.buf[7] = 0x00;

  Can1.write(cmd);
}

void speedControl (uint16_t _id, int32_t _speed) {
  _speed = constrain(_speed, -MAXSPEED, MAXSPEED);
  _speed = (int32_t)((_speed / RPM_TO_DPS) * (GEAR_RATIO * ANGLE_RATIO));
  //  Serial.println(_speed);
  CAN_message_t cmd;
  M1.asInt32_t = _speed;

  cmd.id = _id;
  cmd.buf[0] = 0xA2;
  cmd.buf[1] = 0x00;
  cmd.buf[2] = 0x00;
  cmd.buf[3] = 0x00;
  cmd.buf[4] = M1.asUint8_t[0];
  cmd.buf[5] = M1.asUint8_t[1];
  cmd.buf[6] = M1.asUint8_t[2];
  cmd.buf[7] = M1.asUint8_t[3];

  Can1.write(cmd);
}

void writeACC(uint16_t _id, int32_t _acc) {
  _acc = constrain(_acc, 0, MAXSPEED);
  _acc = (int32_t)((_acc / RPM_TO_DPS) * GEAR_RATIO);
  CAN_message_t cmd;
  M1.asInt32_t = _acc;

  cmd.id = _id;
  cmd.buf[0] = 0x34;
  cmd.buf[1] = 0x00;
  cmd.buf[2] = 0x00;
  cmd.buf[3] = 0x00;
  cmd.buf[4] = M1.asUint8_t[0];
  cmd.buf[5] = M1.asUint8_t[1];
  cmd.buf[6] = M1.asUint8_t[2];
  cmd.buf[7] = M1.asUint8_t[3];

  Can1.write(cmd);
}

void driverInitial() {
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  //  Can1.onReceive(MB0, canMessage);
  //  Can1.onReceive(MB1, canMessage);
  //  Can1.onReceive(MB2, canMessage);
  //  Can1.onReceive(MB3, canMessage);
  Can1.setMBUserFilter(MB0, flWheel, 0xFF);
  Can1.setMBUserFilter(MB1, frWheel, 0xFF);
  Can1.setMBUserFilter(MB0, brWheel, 0xFF);
  Can1.setMBUserFilter(MB1, blWheel, 0xFF);
  Can1.mailboxStatus();

  //  digitalWrite(led, LOW);
  delay(2000);
  writeACC(flWheel, 300);
  writeACC(frWheel, 300);
  writeACC(brWheel, 300);
  writeACC(blWheel, 300);
  delay(200);
  //  digitalWrite(led, HIGH);
}
