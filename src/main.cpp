#include <SPI.h>
#include <Wire.h>
#include <math.h>

#define MPU6050_I2C_ADDRESS 0x68
#define FREQ  30.0 // sample freq in Hz

// global angle, gyro derived
double gSensitivity = 65.5; // for 500 deg/s, check data sheet
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
int16_t accX = 0, accY = 0, accZ = 0;

double gyrXoffs = -281.00, gyrYoffs = 18.00, gyrZoffs = -83.00;


int oldV = 1, newV = 0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 90;
char MotorAction = 'Z';
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
int stoppable = 0;
int count = 0;
char MotorCmd = 'Z';

int rotateLock = 0;
// If rotateLock = 0: FWD
// If rotateLock = 1: rotate1
// If rotateLock = 2: rotate2

int ldone = 1;
int rdone = 1;
int brdone = 1;
int bldone = 1;

bool flag1 = 1;

long rdistance_in_cm;
unsigned long rstart_time = 0;

long ldistance_in_cm;
unsigned long lstart_time = 0;

long brdistance_in_cm;
unsigned long brstart_time = 0;

long bldistance_in_cm;
unsigned long blstart_time = 0;

unsigned long m1_start_time = 0;
unsigned long m2_start_time = 0;
unsigned int m1_done = 1;
unsigned int m2_done = 1;
int not_detected = 1;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define REcho A6 //ultrasonic echo pin RIGHT
#define RTrig A7 //ultrasonic Trigger pin RIGHT
#define LEcho A2 //ultrasonic echo pin LEFT
#define LTrig A3 //ultrasonic Trigger pin LEFT

#define BREcho 32
#define BRTrig 33

#define BLEcho 40
#define BLTrig 41

#define PWMA 12 //Motor A PWM
#define DIRA1 34
#define DIRA2 35 //Motor A Direction
#define PWMB 8   //Motor B PWM
#define DIRB1 37
#define DIRB2 36 //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42 //Motor C Direction
#define PWMD 5   //Motor D PWM
#define DIRD1 A4 //26
#define DIRD2 A5 //27  //Motor D Direction

#define FR_MARGIN 10
#define FL_MARGIN 10
#define BR_MARGIN 20
#define BL_MARGIN 20

// #define ULTRASONIC_TEST

#define MOTORA_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRA1, LOW);  \
    digitalWrite(DIRA2, HIGH); \
    analogWrite(PWMA, pwm);    \
  } while (0)
#define MOTORA_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRA1, LOW); \
    digitalWrite(DIRA2, LOW); \
    analogWrite(PWMA, 0);     \
  } while (0)
#define MOTORA_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRA1, HIGH); \
    digitalWrite(DIRA2, LOW);  \
    analogWrite(PWMA, pwm);    \
  } while (0)

#define MOTORB_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRB1, LOW);  \
    digitalWrite(DIRB2, HIGH); \
    analogWrite(PWMB, pwm);    \
  } while (0)
#define MOTORB_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRB1, LOW); \
    digitalWrite(DIRB2, LOW); \
    analogWrite(PWMB, 0);     \
  } while (0)
#define MOTORB_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRB1, HIGH); \
    digitalWrite(DIRB2, LOW);  \
    analogWrite(PWMB, pwm);    \
  } while (0)

#define MOTORC_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRC1, LOW);  \
    digitalWrite(DIRC2, HIGH); \
    analogWrite(PWMC, pwm);    \
  } while (0)
#define MOTORC_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRC1, LOW); \
    digitalWrite(DIRC2, LOW); \
    analogWrite(PWMC, 0);     \
  } while (0)
#define MOTORC_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRC1, HIGH); \
    digitalWrite(DIRC2, LOW);  \
    analogWrite(PWMC, pwm);    \
  } while (0)

#define MOTORD_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRD1, LOW);  \
    digitalWrite(DIRD2, HIGH); \
    analogWrite(PWMD, pwm);    \
  } while (0)
#define MOTORD_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRD1, LOW); \
    digitalWrite(DIRD2, LOW); \
    analogWrite(PWMD, 0);     \
  } while (0)
#define MOTORD_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRD1, HIGH); \
    digitalWrite(DIRD2, LOW);  \
    analogWrite(PWMD, pwm);    \
  } while (0)

#define SERIAL Serial
#define BTSERIAL Serial3


#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM 2000
#define MIN_PWM 300

int Motor_PWM = 60; //31

int i2c_read(int addr, int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);
  if (n != 1)
  return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
  return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(addr, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
  return (-11);

  return (0);  // return : no error
}

int i2c_write(int addr, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);        // write the start address
  if (n != 1)
  return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
  return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
  return (error);

  return (0);         // return : no error
}


int i2c_write_reg(int addr, int reg, uint8_t data)
{
  int error;
  
  error = i2c_write(addr, reg, &data, 1);
  return (error);
}




long measureDistanceBackL()
{
  long duration;

  if (bldone)
  {
    // reset start_time only if the distance has been measured
    // in the last invocation of the method
    bldone = 0;
    blstart_time = millis();
    digitalWrite(BLTrig, LOW);
  }

  if (millis() > blstart_time + 2)
  {
    digitalWrite(BLTrig, HIGH);
  }

  if (millis() > blstart_time + 10)
  {
    digitalWrite(BLTrig, LOW);
    duration = pulseIn(BLEcho, HIGH);
    bldistance_in_cm = (duration / 2.0) / 29.1;
    bldone = 1;
  }

  return bldistance_in_cm;
}

long measureDistanceBackR()
{
  long duration;

  if (brdone)
  {
    // reset start_time only if the distance has been measured
    // in the last invocation of the method
    brdone = 0;
    brstart_time = millis();
    digitalWrite(BRTrig, LOW);
  }

  if (millis() > brstart_time + 2)
  {
    digitalWrite(BRTrig, HIGH);
  }

  if (millis() > brstart_time + 10)
  {
    digitalWrite(BRTrig, LOW);
    duration = pulseIn(BREcho, HIGH);
    brdistance_in_cm = (duration / 2.0) / 29.1;
    brdone = 1;
  }
}

long measureDistanceRight()
{
  long duration;

  if (rdone)
  {
    // reset start_time only if the distance has been measured
    // in the last invocation of the method
    rdone = 0;
    rstart_time = millis();
    digitalWrite(RTrig, LOW);
  }

  if (millis() > rstart_time + 2)
  {
    digitalWrite(RTrig, HIGH);
  }

  if (millis() > rstart_time + 10)
  {
    digitalWrite(RTrig, LOW);
    duration = pulseIn(REcho, HIGH);
    rdistance_in_cm = (duration / 2.0) / 29.1;
    rdone = 1;
  }
}

long measureDistanceLeft()
{
  long duration;

  if (ldone)
  {
    // reset start_time only if the distance has been measured
    // in the last invocation of the method
    ldone = 0;
    lstart_time = millis();
    digitalWrite(LTrig, LOW);
  }

  if (millis() > lstart_time + 2)
  {
    digitalWrite(LTrig, HIGH);
  }

  if (millis() > lstart_time + 10)
  {
    digitalWrite(LTrig, LOW);
    duration = pulseIn(LEcho, HIGH);
    ldistance_in_cm = (duration / 2.0) / 29.1;
    ldone = 1;
  }
}

//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1() //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2() // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

void calibrate(){

  int x;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t i2cData[6]; 
  int num = 500;
  uint8_t error;

  for (x = 0; x < num; x++){

    error = i2c_read(MPU6050_I2C_ADDRESS, 0x43, i2cData, 6);
    if(error!=0)
    return;

    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;

//  Serial.println("Calibration result:");
//  Serial.print(gyrXoffs);
//  Serial.print(", ");
//  Serial.print(gyrYoffs);
//  Serial.print(", ");
//  Serial.println(gyrZoffs);
  
} 


void read_sensor_data(){
 uint8_t i2cData[14];
 uint8_t error;
 // read imu data
 error = i2c_read(MPU6050_I2C_ADDRESS, 0x3b, i2cData, 14);
 if(error!=0)
 return;

 // assemble 16 bit sensor data
 accX = ((i2cData[0] << 8) | i2cData[1]);
 accY = ((i2cData[2] << 8) | i2cData[3]);
 accZ = ((i2cData[4] << 8) | i2cData[5]);

 gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
 gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
 gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;
 
}

void REPORT_GYRO()
{
      delay(30);
      Serial.print(gx, 2);
      Serial.print(", ");
      Serial.print(gy, 2);
      Serial.print(", ");
      Serial.println(gz, 2);
}

// Task 3 custom code
void UART_Control()
{
  if (SERIAL.available())
  {
    MotorCmd = Serial.read();
  }
    // Serial.println(MotorCmd);
    switch (MotorCmd)
    { 
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK();     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    // MPU6050 support
    case '.':  REPORT_GYRO(); break;
    }
    delay(50);
    MotorCmd = 'Z';

}

//Where the program starts
void setup()
{
  uint8_t sample_div;
  SERIAL.begin(115200); // USB serial setup CHANGE BACK TO 115200
  SERIAL.println("Start");
  STOP();              // Stop the robot
  Serial3.begin(9600); // BT serial setup
                       //Pan=PL4=>48, Tilt=PL5=>47
  servo_pan.attach(48);
  servo_tilt.attach(47);

  //Setup Voltage detector
  pinMode(A0, INPUT);
  //initialize ultrasonic echo and trigger pins
  pinMode(REcho, INPUT);
  pinMode(LEcho, INPUT);
  pinMode(RTrig, OUTPUT);
  pinMode(LTrig, OUTPUT);
  pinMode(BREcho, INPUT);
  pinMode(BRTrig, OUTPUT);
  pinMode(BLEcho, INPUT);
  pinMode(BLTrig, OUTPUT);
  // Wire.begin();
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x6b, 0x00);

  // CONFIG:
  // Low pass filter samples, 1khz sample rate
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x1a, 0x01);

  // GYRO_CONFIG:
  // 500 deg/s, FS_SEL=1
  // This means 65.5 LSBs/deg/s
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);

  // CONFIG:
  // set sample rate
  // sample rate FREQ = Gyro sample rate / (sample_div + 1)
  // 1kHz / (div + 1) = FREQ  
  // reg_value = 1khz/FREQ - 1
  sample_div = 1000 / FREQ - 1;
  i2c_write_reg (MPU6050_I2C_ADDRESS, 0x19, sample_div);
  calibrate();


}

void loop()
{

  // run the code in every 20ms

    int error;
    double dT;
    double ax, ay, az;
    unsigned long start_time, end_time;
    start_time = millis();
    

    //constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);

    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);

    read_sensor_data();

    // angles based on accelerometer
    ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
    ax = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

    // angles based on gyro (deg/s)
    gx = gx + gyrX / FREQ;
    gy = gy - gyrY / FREQ;
    gz = gz + gyrZ / FREQ;

    // complementary filter
    // tau = DT*(A)/(1-A)
    // = 0.48sec
    gx = gx * 0.96 + ax * 0.04;
    gy = gy * 0.96 + ay * 0.04;

    UART_Control(); //get USB and BT serial data
    end_time = millis();
    // delay(((1/FREQ) * 1000) - (end_time - start_time));

}
