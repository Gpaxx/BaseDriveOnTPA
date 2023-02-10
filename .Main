

#include <FlexCAN_T4.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <I2Cdev.h>
MPU6050 mpu;

#define INTERRUPT_PIN 17               //Interrupt pin imu

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can1;

//id of motor 0x140+id
#define flWheel 0x141
#define frWheel 0x142
#define brWheel 0x143
#define blWheel 0x144
#define MAXSPEED    90
#define GEAR_RATIO  10
#define RPM_TO_DPS  0.166666666666666666666666666667
#define ANGLE_RATIO 100


#include <PS4Serial.h>
#include <Encoder.h>

#define fcEncoder_A 24
#define fcEncoder_B 25
Encoder fcEncoder(fcEncoder_A, fcEncoder_B);

#define blEncoder_A 26
#define blEncoder_B 27
Encoder blEncoder(blEncoder_A, blEncoder_B);

#define brEncoder_A 28
#define brEncoder_B 29
Encoder brEncoder(brEncoder_A, brEncoder_B);

PS4Serial joy;


bool dmpReady = false;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint8_t fifoBuffer[64];
uint16_t packetSize;
uint16_t fifoCount;      // count of all bytes currently in FIFO

float ypr[3];
float ypr_readable[3];
float ypr_readable_map = 0.0f;
float yaw_map_circle = 0.0f;

float SxCompensate = 0.0f;
float SyCompensate = 0.0f;


float SxCompensateRST = 0.0f;
float SyCompensateRST = 0.0f;


int omega;
float dx , dy;
volatile long ENC1, ENC2 , ENC3;
float Direction_test ;
float ThetaRobot = 0;
bool state_test;
int8_t NewLx = 0;
int8_t NewLy = 0;
int8_t Lx = 0;
int8_t Ly = 0;
float alpha = 0;
float alphaDegree = 0;
bool stateJoy = false ;
bool stateAnalog = false;
bool stateAnalogx = false;
bool stateAnalogy = false;
long currentPluseFC = -999;
long currentPluseBL = -999;
long currentPluseBR = -999;

Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y,U z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container

volatile bool mpuInterrupt = false;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  joy.setPort(&Serial1);
  driverInitial();
//  delay(1000);
  imu_init();
  
}



void loop() {
  ///   put your main code here, to run repeatedly:
 
    imu_run();  
    joy.getAnalog(joy.PS4A_L2);
    DriveMotor(2);

}
