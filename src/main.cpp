#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 90;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
int stoppable = 0;
int ldone = 1;
int rdone = 1;

long rdistance_in_cm;
unsigned long rstart_time = 0;
long ldistance_in_cm;
unsigned long lstart_time = 0;
long lfilterArray[20];
long rfilterArray[20];

unsigned long m1_start_time = 0;
unsigned long m2_start_time = 0;
unsigned int m1_done = 1;
unsigned int m2_done = 1;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define REcho A6  //ultrasonic echo pin RIGHT
#define RTrig A7  //ultrasonic Trigger pin RIGHT
#define LEcho A2  //ultrasonic echo pin LEFT
#define LTrig A3  //ultrasonic Trigger pin LEFT

#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 1900;

long measureDistanceRight(){
  long duration;
  
  if (rdone) {     
    // reset start_time only if the distance has been measured 
    // in the last invocation of the method
    rdone = 0;
    rstart_time = millis();
    digitalWrite(RTrig, LOW);
  }
  
  if (millis() > rstart_time + 2) { 
    digitalWrite(RTrig, HIGH);
  }
  
  if (millis() > rstart_time + 10) {
    digitalWrite(RTrig, LOW);
    duration = pulseIn(REcho, HIGH);
    rdistance_in_cm = (duration / 2.0) / 29.1;
    rdone = 1;
  }

  return rdistance_in_cm;
  
}

long measureDistanceLeft(){
  long duration;
  
  if (ldone) {     
    // reset start_time only if the distance has been measured 
    // in the last invocation of the method
    ldone = 0;
    lstart_time = millis();
    digitalWrite(LTrig, LOW);
  }
  
  if (millis() > lstart_time + 2) { 
    digitalWrite(LTrig, HIGH);
  }
  
  if (millis() > lstart_time + 10) {
    digitalWrite(LTrig, LOW);
    duration = pulseIn(LEcho, HIGH);
    ldistance_in_cm = (duration / 2.0) / 29.1;
    ldone = 1;
  }

  return ldistance_in_cm;
  
}

long measureAndFilterDistanceL()
{
  for (int sample = 0; sample < 20; sample++) {
      lfilterArray[sample] = measureDistanceLeft();
    //delay(30); // to avoid untrasonic interfering
  }
  for (int i = 0; i < 19; i++) {
    for (int j = i + 1; j < 20; j++) {
      if (lfilterArray[i] > lfilterArray[j]) {
        float swap = lfilterArray[i];
        lfilterArray[i] = lfilterArray[j];
        lfilterArray[j] = swap;
      }
    }
  }
  long sum = 0;
  for (int sample = 5; sample < 15; sample++) {
    sum += lfilterArray[sample];
  }
  return sum / 10;

}


long measureAndFilterDistanceR()
{
  for (int sample = 0; sample < 20; sample++) {
      rfilterArray[sample] = measureDistanceRight();
    //delay(30); // to avoid untrasonic interfering
  }
  for (int i = 0; i < 19; i++) {
    for (int j = i + 1; j < 20; j++) {
      if (rfilterArray[i] > rfilterArray[j]) {
        float swap = rfilterArray[i];
        rfilterArray[i] = rfilterArray[j];
        rfilterArray[j] = swap;
      }
    }
  }
  long sum = 0;
  for (int sample = 5; sample < 15; sample++) {
    sum += rfilterArray[sample];
  }
  return sum / 10;

}



//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
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
  MOTORA_FORWARD(Motor_PWM+10); 
  MOTORB_FORWARD(Motor_PWM+10);
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
  MOTORA_BACKOFF(Motor_PWM+10); 
  MOTORB_BACKOFF(Motor_PWM+10);
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
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
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
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
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

void UART_Control()
{

  String myString;
  char BT_Data = 0;

  Motor_PWM = 1850;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */

  if (SERIAL.available())
  {
    if (stoppable)
    {
      STOP();
      return;
    }
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
      measureAndFilterDistanceL();
      measureAndFilterDistanceR();

      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println(ldistance_in_cm);
      display.println(rdistance_in_cm);
      display.display();

      if (ldistance_in_cm < 1 || rdistance_in_cm < 1)
      {
        STOP();
        return;
      }
    }

    if (window_size == 0)
    {
      Serial.println("STOP");
      STOP();
      return;
    }


    if (ldistance_in_cm < 13 || rdistance_in_cm < 13)
    {
      if (ldistance_in_cm > 800)
      {
        LEFT_2();
        delay(20);
        STOP();
        return;
      }
      else if (rdistance_in_cm > 800)
      {
        RIGHT_2();
        delay(20);
        STOP();
        return;
      }
    }

    // R1 = Left
    // R2 = Right

    // Near field mode
    if (ldistance_in_cm < 10 || rdistance_in_cm < 10)
    {
      // Promixity Guide Mode
      Serial.print("PGM on ");
      measureAndFilterDistanceL();
      measureAndFilterDistanceR();
      delay(50);
      Serial.print("Ld = ");
      Serial.print(ldistance_in_cm);
      Serial.print(" Rd = ");
      Serial.print(rdistance_in_cm);
      Motor_PWM = 1900;

      if (ldistance_in_cm == rdistance_in_cm)
      {
        ADVANCE();
        delay(15);
        STOP();
        measureAndFilterDistanceL();
      measureAndFilterDistanceR();
      }

      if (ldistance_in_cm > rdistance_in_cm)
      {
        Serial.println(" -> R1");
        // rotate_1();
        LEFT_2();
        delay(9);
        STOP();
              measureAndFilterDistanceL();
      measureAndFilterDistanceR();

      }
      if (rdistance_in_cm > ldistance_in_cm)
      {
         Serial.println(" -> R2");
        RIGHT_2();
        delay(10);
        STOP();
              measureAndFilterDistanceL();
      measureAndFilterDistanceR();

      }

    }

    if (m2_done)
    {
      STOP();
      Motor_PWM = 1850;
      m2_done = 0;
      m2_start_time = millis();
    }
    else
    {
      if (m1_done)
      {
        STOP();
        m1_done = 0;
        m1_start_time = millis();
      }
      else
      {
        if (pan < 86)
        {
          Serial.println("R2");
          Motor_PWM = 1850;
          rotate_2();
        }
        else if (pan > 94)
        {
           Serial.println("R1");
            Motor_PWM = 1850;
          rotate_1();
        }
        else
        {
          Motor_PWM = 1900;
          Serial.println("FWD");
          ADVANCE();
         
        }
      }

      if (millis() > m1_start_time + 60)
      {
        m1_done = 1;
      }

    }

    if (millis() > m2_start_time + 500)
    {
      m2_done = 1;
      Serial.println("STOP");
      Motor_PWM = 1850;
      STOP();
    }



    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println(ldistance_in_cm);
      display.println(rdistance_in_cm);
      display.display();
    }
  }

  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}



/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0);
    if(newV!=oldV) {
      if (newV > 0)
      {
        if (newV > 2.5)
        {
          stoppable = 1;
        }
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Power = ");
        display.print((float) (5*newV / 51)*(5*newV / 51)/50);
        display.display();
      }
      else
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("AI Robot");
        display.display();
      }
      if (!Serial3.available()) {
        Serial3.println(newV);
        Serial.println(newV);
      }
    }
    oldV=newV;
}




//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup CHANGE BACK TO 115200
  SERIAL.println("Start");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
   servo_pan.attach(48);
   servo_tilt.attach(47);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot");
  display.display();


  //Setup Voltage detector
  pinMode(A0, INPUT);
  //initialize ultrasonic echo and trigger pins
  pinMode(REcho,INPUT);
  pinMode(LEcho, INPUT);
  pinMode(RTrig,OUTPUT);
  pinMode(LTrig,OUTPUT);
}

void loop()
{  
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
    UART_Control(); //get USB and BT serial data

    //constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);
    
    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);
  }if (voltCount>=5){
    voltCount=0;
    sendVolt();
  }
}
