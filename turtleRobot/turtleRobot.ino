#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <PID_v1.h>

// 0.04 radius van band
#define LeftM1 5
#define LeftM2 6
#define RightM1 7
#define RightM2 8
#define LeftPwm 9
#define RightPwm 10
#define EncoderLeftA 2
#define EncoderLeftB 3
#define EncoderRightA 20
#define EncoderRightB 21
double DistancePerPulse = 0.000418879;

float x;
float z;

double PkLeft = 30;
double IkLeft = 10;
double DkLeft = 0.01;

double SetpointLeft, InputLeft, OutputLeft;
PID PIDLeft(&InputLeft, &OutputLeft, &SetpointLeft, PkLeft, IkLeft, DkLeft, DIRECT);

double PkRight = 30;
double IkRight = 10;
double DkRight = 0.01;

double SetpointRight, InputRight, OutputRight;
PID PIDRight(&InputRight, &OutputRight, &SetpointRight, PkRight, IkRight, DkRight, DIRECT);

float demandLeft = 2.5;
float demandRight = 2.5;

ros::NodeHandle nh;

void velCallback( const geometry_msgs::Twist& vel){
  x = vel.linear.x;
  z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

volatile int encoderLeftPos = 0;
volatile int encoderLeftPosPrev = 0;
volatile int encoderRightPos = 0;
volatile int encoderRightPosPrev = 0;
volatile int pulseCountLeft;
volatile int pulseCountRight;

double velocityLeft;
double velocityRight;

unsigned long current = 0;
unsigned long prev = 0;
  
void setup()
{
  pinMode(LeftM1, OUTPUT);
  pinMode(LeftM2, OUTPUT);
  pinMode(RightM1, OUTPUT);
  pinMode(RightM2, OUTPUT);
  pinMode(LeftPwm, OUTPUT);
  pinMode(RightPwm, OUTPUT);
  pinMode(EncoderLeftA, INPUT_PULLUP);
  pinMode(EncoderLeftB, INPUT_PULLUP);
  pinMode(EncoderRightA, INPUT_PULLUP);
  pinMode(EncoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncoderLeftA), doEncoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderLeftB), doEncoderLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderRightA), doEncoderRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderRightB), doEncoderRightB, CHANGE);

  PIDLeft.SetMode(AUTOMATIC);
  PIDLeft.SetOutputLimits(-240, 240);
  PIDLeft.SetSampleTime(10);

  PIDRight.SetMode(AUTOMATIC);
  PIDRight.SetOutputLimits(-240, 240);
  PIDRight.SetSampleTime(10);
  
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(115200);
}
 
void loop()
{
  
  current = millis();
  if (current - prev >= 10){
    // Serial.print("demand: ");
    // Serial.println(demandLeft);
    // Serial.print("posLEFT: ");
    // Serial.println(encoderLeftPos);
    // Serial.print("posRIGHT: ");
    // Serial.println(encoderRightPos);
    // Serial.println("-------------------");
    calculateVel();
    calculatePID();
    drive();
    
  }
  if (current >= 30000){
    demandLeft = 0;
    demandRight = 0;
  }

  nh.spinOnce();
  
}

void calculateVel(){
    pulseCountLeft = encoderLeftPos - encoderLeftPosPrev;
    velocityLeft = pulseCountLeft * DistancePerPulse;
    encoderLeftPosPrev = encoderLeftPos;
    velocityLeft = velocityLeft * 100;

    pulseCountRight = encoderRightPos - encoderRightPosPrev;
    velocityRight = pulseCountRight * DistancePerPulse;
    velocityRight = velocityRight * 100;
    encoderRightPosPrev = encoderRightPos;

    Serial.print("velWheelLEFT: ");
    Serial.println(velocityLeft);
    Serial.print("velWheelRight: ");
    Serial.println(velocityRight);
    Serial.println("-------------------");
}

void calculatePID(){
    SetpointLeft = demandLeft;
    InputLeft = velocityLeft;
    PIDLeft.Compute();

    SetpointRight = demandRight;
    InputRight = velocityRight;
    PIDRight.Compute();
}

void drive(){
  if(demandLeft == 0 && velocityLeft == 0){
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, 0);
  }
  else{
    if (OutputLeft > 0){
      digitalWrite(LeftM1, HIGH);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, abs(OutputLeft));
    }else if (OutputLeft < 0){
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, HIGH);
      analogWrite(LeftPwm, abs(OutputLeft));
    }
    else{
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, 0);
      
    }
  }
    if(demandRight == 0 && velocityRight == 0){
      digitalWrite(RightM1, LOW);
      digitalWrite(RightM2, LOW);
      analogWrite(RightPwm, 0);
    }
    else{
      if (OutputRight > 0){
      digitalWrite(RightM1, HIGH);
      digitalWrite(RightM2, LOW);
      analogWrite(RightPwm, abs(OutputRight));
    }else if (OutputRight < 0){
      digitalWrite(RightM1, LOW);
      digitalWrite(RightM2, HIGH);
      analogWrite(RightPwm, abs(OutputRight));
    }
    else{
      digitalWrite(RightM1, LOW);
      digitalWrite(RightM2, LOW);
      analogWrite(RightPwm, 0);      
    }
  } 
}

void doEncoderLeftA(){
  if (digitalRead(EncoderLeftA) == HIGH) {
    if(digitalRead(EncoderLeftB) == LOW){
        encoderLeftPos = encoderLeftPos + 1;
      }
      else{
        encoderLeftPos = encoderLeftPos - 1;
      }
    }
    else{
      if(digitalRead(EncoderLeftB) == HIGH){
        encoderLeftPos = encoderLeftPos + 1;
      }
      else{
        encoderLeftPos = encoderLeftPos - 1;
      }
    }
}

void doEncoderLeftB(){
  if (digitalRead(EncoderLeftB) == HIGH) {
    if(digitalRead(EncoderLeftA) == HIGH){
        encoderLeftPos = encoderLeftPos + 1;
      }
      else{
        encoderLeftPos = encoderLeftPos - 1;
      }
    }
    else{
      if(digitalRead(EncoderLeftA) == LOW){
        encoderLeftPos = encoderLeftPos + 1;
      }
      else{
        encoderLeftPos = encoderLeftPos - 1;
      }
    }
}

void doEncoderRightA(){
  if (digitalRead(EncoderRightA) == HIGH) {
    if(digitalRead(EncoderRightB) == LOW){
        encoderRightPos = encoderRightPos + 1;
      }
      else{
        encoderRightPos = encoderRightPos - 1;
      }
    }
    else{
      if(digitalRead(EncoderRightB) == HIGH){
        encoderRightPos = encoderRightPos + 1;
      }
      else{
        encoderRightPos = encoderRightPos - 1;
      }
    }
}

void doEncoderRightB(){
  if (digitalRead(EncoderRightB) == HIGH) {
    if(digitalRead(EncoderRightA) == HIGH){
        encoderRightPos = encoderRightPos + 1;
      }
      else{
        encoderRightPos = encoderRightPos - 1;
      }
    }
    else{
      if(digitalRead(EncoderRightA) == LOW){
        encoderRightPos = encoderRightPos + 1;
      }
      else{
        encoderRightPos = encoderRightPos - 1;
      }
    }
}
