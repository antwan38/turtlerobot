#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <PID_v1.h>


#define LeftM1 5
#define LeftM2 6
#define RightM1 7
#define RightM2 8
#define LeftPwm 9
#define RightPwm 10
#define EncoderLeftA 2
#define EncoderLeftB 3
#define EncoderRightA 12
#define EncoderRightB 11

float x;
float z;

double Pk = 1;
double Ik = 0;
double Dk = 0.01;

double Setpoint, Input, Output;
PID PIDLeft(&Input, &Output, &Setpoint, Pk, Ik, Dk, DIRECT);

float demand = 14500;

ros::NodeHandle nh;

void velCallback( const geometry_msgs::Twist& vel){
  x = vel.linear.x;
  z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

volatile int encoderLeftPos = 0;
volatile int encoderRightPos = 0; 

unsigned long current =0;
unsigned long prev =0;
  
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

  PIDLeft.SetMode(AUTOMATIC);
  PIDLeft.SetOutputLimits(-100, 100);
  PIDLeft.SetSampleTime(10);
  
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(115200);
}
 
void loop()
{
  
  current = millis();
  if (current - prev >= 10){
    prev = current;
    if (Serial.available()>0){
      char c = Serial.read();

      if (c == 's'){
        demand = 14500;
      }else if (c == 'p'){
        demand = 0;
      }
    }
    Serial.print("demand: ");
    Serial.println(demand);
    Serial.print("pos: ");
    Serial.println(encoderLeftPos);
    Serial.println("-------------------");
    Setpoint = demand;
    Input = encoderLeftPos;
    PIDLeft.Compute();

    if (Output > 0){

      digitalWrite(LeftM1, HIGH);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, abs(Output));
    }else if (Output < 0){
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, HIGH);
      analogWrite(LeftPwm, abs(Output));
    }
    else{
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, 0);
      
    }

  }
  
  
  //Serial1.print(x);
  //Serial1.print(" , ");
  //Serial1.print(z);

  nh.spinOnce();
  
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
