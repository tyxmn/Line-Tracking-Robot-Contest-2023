#include <QTRSensors.h>
#include <Servo.h>
//////////////// TUNNING //////////////////
#define KP 0.1   ///// 0.1
#define KD 0.6   ///// 0.5 
///////////////////////////////////////////

////////// DEFAULT MOTOR SPEED ////////////
#define M1_minumum_speed 45 //60
#define M2_minumum_speed 45 // 60
#define M1_maksimum_speed 60 //70
#define M2_maksimum_speed 60 //70
///////////////////////////////////////////

////////// Soft MOTOR SPEED ////////////
#define M1_minumumSoft_speed 100       //50
#define M2_minumumSoft_speed 100      //505
#define M1_maksimumSoft_speed 120      //90
#define M2_maksimumSoft_speed 120      //90
///////////////////////////////////////////

////////// Grind1 MOTOR SPEED  //////////
#define FCROSS_1_M1_minumum_speed 25
#define FCROSS_1_M2_minumum_speed 25
#define FCROSS_1_M1_maksimum_speed  35
#define FCROSS_1_M2_maksimum_speed  35
//////////////////////////////////////////3

////////// Grind2 MOTOR SPEED  //////////
#define FCROSS_2_M1_minumum_speed 75
#define FCROSS_2_M2_minumum_speed 75
#define FCROSS_2_M1_maksimum_speed 150
#define FCROSS_2_M2_maksimum_speed 150
///////////////////////////////////////////

////////// Grind3 MOTOR SPEED  //////////
#define FCROSS_3_M1_minumum_speed 60
#define FCROSS_3_M2_minumum_speed 60
#define FCROSS_3_M1_maksimum_speed 120
#define FCROSS_3_M2_maksimum_speed 120
///////////////////////////////////////////


////////// STOP_MOTOR SPEED  //////////
#define STOP_1_M1_minumum_speed 0
#define STOP_1_M2_minumum_speed 0
#define STOP_1_M1_maksimum_speed  0
#define STOP_1_M2_maksimum_speed  0
//////////////////////////////////////////

////////////////// SENSORS ////////////////
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN A7
#define DEBUG 0
///////////////////////////////////////////

Servo ESC;
//////Button////////////
int Bt1 = 8;
int Bt2 = 9;

int LED_1 = 13;


int V1;
int V2;
int Ls1;
int Ls2;
bool StateRun;
////////////////// MOTOR //////////////////
///RIGHT
int PWMA = 5;
int AIN1 = 2;
int AIN2 = 4;

//LEFT
int PWMB = 6;
int BIN1 = 3;
int BIN2 = 7;



///////////////ตัวแปล FCROSS เพื่อปรับความเร็ว//////////////////
int FCROSS = 0;
////////////////// MOTOR_Speed_Change //////////////////
int M1_MIN;
int M2_MIN;
int M1_MAX;
int M2_MAX;
///////////// INITIAL VALUES (ค่าเมื่อเจอเส้นกริด) //////////////
int S0_DT = 1400;
int S1_DT = 1400;
int S6_DT = 1400;
int S7_DT = 1400;
///////////////////////////////////////////
int S2_DT = 1400;
int S3_DT = 1400;
int S4_DT = 1400;
int S5_DT = 1400;
////////////////////////////////////////////

///////////////////////////////////////////

QTRSensorsRC qtrrc((unsigned char[]) {
  11, 10, A0, A1, A2, A3, A4, A5
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  Serial.begin(9600);
  ESC.attach(12, 1000, 2000);
  ESC.write(0);
  delay(500);
  auto_calibration();           // ใช้ค่าจูนอัตโนมัติ
  //  manual_calibration();       // ใช้ค่าจูนด้วยตนเอง



  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(Bt1, INPUT);
  pinMode(Bt2, INPUT);
}

int lastError = 0;
int last_proportional = 0;
int integral = 0;

void loop() {
  unsigned int sensors[8];
  float position = qtrrc.readLine(sensors);
  qtrrc.read(sensorValues);

  V1 = digitalRead(Bt1);
  V2 = digitalRead(Bt2);

  int error = position - 3500;

  float motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  ////////Grid_Sensor///////////    PIN
  int ss0 = sensorValues[0];      // 0
  int ss1 = sensorValues[1];      // 1
  int ss2 = sensorValues[2];      // A2
  int ss3 = sensorValues[3];      // A1
  int ss4 = sensorValues[4];      // 2
  int ss5 = sensorValues[5];      // 4
  int ss6 = sensorValues[6];      // A5
  int ss7 = sensorValues[7];      // A4

  int leftMotorSpeed = M1_MIN + motorSpeed;
  int rightMotorSpeed = M2_MIN - motorSpeed;

  if (leftMotorSpeed > M1_MAX ) leftMotorSpeed = M1_MAX;
  if (rightMotorSpeed > M2_MAX ) rightMotorSpeed = M2_MAX;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;

  if (V1 == 0 && Ls1 == 1 && StateRun == 0 ) {
    delay(400);
    StateRun = 1;
  }
  if (StateRun == 1) {
    motor_left_speed( leftMotorSpeed);
    motor_right_speed(rightMotorSpeed);
  } else {
    motor_left_speed(0);
    motor_right_speed(0);
  }

  //  Serial.println(FCROSS);
  //////////////////////////// เช็คเส้นกริด //////////////////////////////////
  if (ss0 > S0_DT && ss1 > S1_DT && ss6 > S6_DT && ss7 > S7_DT) {
    if (FCROSS == 0) {
      FCROSS = 1;
    }
  } else if (ss0 < S0_DT && ss1 < S1_DT && ss6 > S6_DT && ss7 > S7_DT && FCROSS == 1) {
    Forwerd();
    delay(100);
    FCROSS = 2;
  } else if (ss0 < S0_DT && ss1 < S1_DT && ss6 > S6_DT && ss7 > S7_DT && FCROSS == 2) {
    FCROSS = 3;
  } else if (ss0 < S0_DT && ss1 < S1_DT && ss2 < S2_DT && ss5 < S5_DT && ss6 < S6_DT && ss7 < S7_DT && ss3 < S3_DT && ss4 < S4_DT && FCROSS == 3) {
    FCROSS = 4;
  } else if (ss0 < S0_DT && ss1 < S1_DT && ss5 > S5_DT && ss6 > S6_DT && FCROSS == 4) {
    FCROSS = 5;
  }   else  if (ss7 > S7_DT &&position <2000&& FCROSS == 5 ) {        ////////////โค้งเล็ก/////////
    motor_Stop();
    motor_right_speed(rightMotorSpeed);

  }



  if (ss0 > S0_DT && ss1 > S1_DT && ss6 > S6_DT && ss7 > S7_DT && FCROSS == 5) {
    FCROSS = 6;
  }







  if (FCROSS == 0 ) {
    ESC.write(0);
    M1_MIN = M1_minumum_speed;
    M2_MIN = M2_minumum_speed;
    M1_MAX = M1_maksimum_speed;
    M2_MAX = M2_maksimum_speed;
  } else if (FCROSS == 1 ) {
    ESC.write(20);
    M1_MIN = FCROSS_1_M1_minumum_speed;
    M2_MIN = FCROSS_1_M2_minumum_speed;
    M1_MAX = FCROSS_1_M1_maksimum_speed;
    M2_MAX = FCROSS_1_M2_maksimum_speed;
  } else if (FCROSS == 2) {
    ESC.write(30);
    Turn90_L();
  }  else if (FCROSS == 3) {
    ESC.write(50);
    M1_MIN = FCROSS_2_M1_minumum_speed;
    M2_MIN = FCROSS_2_M2_minumum_speed;
    M1_MAX = FCROSS_2_M1_maksimum_speed;
    M2_MAX = FCROSS_2_M2_maksimum_speed;
  }  else if (FCROSS == 4) {
    motor_left_speed(20);
    motor_right_speed(70);
  } else if (FCROSS == 5) {
    ESC.write(50);
    M1_MIN = M1_minumum_speed;
    M2_MIN = M2_minumum_speed;
    M1_MAX = M1_maksimum_speed;
    M2_MAX = M2_maksimum_speed;
  } else if (FCROSS == 6) {
    M1_MIN = STOP_1_M1_minumum_speed;
    M2_MIN = STOP_1_M2_minumum_speed;
    M1_MAX = STOP_1_M1_maksimum_speed;
    M2_MAX = STOP_1_M2_maksimum_speed;
    ESC.write(0);
  }





  Ls1 = V1;
  Ls2 = V2;

}
void Forwerd() {
  digitalWrite(AIN1, HIGH);       // Right
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 20);
  digitalWrite(BIN1, HIGH);       // Left
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 20);
}
void Turn90_L() {
  digitalWrite(AIN1, 0);       // Right
  digitalWrite(AIN2, 1);
  analogWrite(PWMA, 70);
  digitalWrite(BIN1, 1);       // Left
  digitalWrite(BIN2, 0);
  analogWrite(PWMB, 70);
}
void motor_Stop()
{
  digitalWrite(AIN1, 0);       // BACKWARD
  digitalWrite(AIN2, 1);
  analogWrite(PWMA, 150 );
 
}
void Turn90_R() {
  digitalWrite(AIN1, 1);       // Right
  digitalWrite(AIN2, 0);
  analogWrite(PWMA, 70);
  digitalWrite(BIN1, 0);       // Left
  digitalWrite(BIN2, 1);
  analogWrite(PWMB, 70);
}


void motor_left_speed(int spdmotor1)
{
  digitalWrite(BIN1, HIGH);       // FORWARD
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, spdmotor1);
}
void motor_right_speed(int spdmotor2)
{
  digitalWrite(AIN1, HIGH);       // FORWARD
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, spdmotor2);
}

void auto_calibration() {
  qtrrc.calibrate(QTR_EMITTERS_ON);

  qtrrc.calibratedMinimumOn[0] = 814;
  qtrrc.calibratedMinimumOn[1] = 617;
  qtrrc.calibratedMinimumOn[2] = 528;
  qtrrc.calibratedMinimumOn[3] = 509;
  qtrrc.calibratedMinimumOn[4] = 566;
  qtrrc.calibratedMinimumOn[5] = 556;
  qtrrc.calibratedMinimumOn[6] = 725;
  qtrrc.calibratedMinimumOn[7] = 666;


  qtrrc.calibratedMaximumOn[0] = 2500;
  qtrrc.calibratedMaximumOn[1] = 2500;
  qtrrc.calibratedMaximumOn[2] = 2500;
  qtrrc.calibratedMaximumOn[3] = 2500;
  qtrrc.calibratedMaximumOn[4] = 2500;
  qtrrc.calibratedMaximumOn[5] = 2500;
  qtrrc.calibratedMaximumOn[6] = 2500;
  qtrrc.calibratedMaximumOn[7] = 2500;
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {
  int i;
  for (i = 0; i < 200; i++)
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }

  if (DEBUG == 1) {
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
}
