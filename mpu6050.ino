void imu_init() {
  // initialize i2c
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(0x68);
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(155);
  mpu.setYGyroOffset(-83);
  mpu.setZGyroOffset(6);

  mpu.setXAccelOffset(-5420);
  mpu.setYAccelOffset(-286);
  mpu.setZAccelOffset(1166);


  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

float mpu_map_yaw(float value) {
  static float yaw_degree = 0;

  if (value >= 0 && value <= 90) {
    yaw_degree = map(value * -1, 0, -90, 90, 0);
  }
  else if (value > 90 && value <= 180) {
    yaw_degree = map(value, 90, 180, 360, 270);
  }
  else if (value >= -90 && value <= 0) {
    yaw_degree = map(value * -1, 0, 90, 90, 180);
  }
  else if (value >= -90 && value <= 0) {
    yaw_degree = map(value * -1, 0, 90, 90, 180);
  }
  else if (value  <= -90 && value >= -180) {
    yaw_degree = map(value * -1, 90, 180, 180, 270);
  }

  return yaw_degree;
}



void dmpDataReady() {
  mpuInterrupt = true;
}

void imu_run() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr_readable[0] = (-ypr[0] * RAD_TO_DEG);

    if (ypr_readable[0] < 180 && ypr_readable[0] >= 0) {
      ypr_readable_map = ypr_readable[0];
    }
    else if (ypr_readable[0] >= -180 && ypr_readable[0] < 0 ) {
      ypr_readable_map = map(ypr_readable[0] , -180, 0 , 180 , 360 );
    }
    
    yaw_map_circle = mpu_map_yaw(ypr_readable[0]);

  }
}
