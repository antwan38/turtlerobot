#include <PID_v1.h>
#include <ArduinoJson.h>

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

JsonDocument robotValues;

double DistancePerPulse = 0.000418879;
double LengthBetweenWheels = 0.25;

double xRobot;
double yRobot;
double theta;

double piLiniar;
double piAngular;

char base_link[] = "/base_link";
char odom[] = "/odom";

double PkLeft = 100;
double IkLeft = 0;
double DkLeft = 0.00;

double SetpointLeft, InputLeft, OutputLeft;
PID PIDLeft(&InputLeft, &OutputLeft, &SetpointLeft, PkLeft, IkLeft, DkLeft, DIRECT);

double PkRight = 100;
double IkRight = 0;
double DkRight = 0.00;

double SetpointRight, InputRight, OutputRight;
PID PIDRight(&InputRight, &OutputRight, &SetpointRight, PkRight, IkRight, DkRight, DIRECT);

double piCalcLeft = 0;
double piCalcRight = 0;

volatile int encoderLeftPos = 0;
volatile int encoderLeftPosPrev = 0;
volatile int encoderRightPos = 0;
volatile int encoderRightPosPrev = 0;
volatile int pulseCountLeft;
volatile int pulseCountRight;

double velocityLeft;
double velocityRight;

double robotVel;
double robotAngVel;

unsigned long current = 0;
unsigned long prev = 0;
unsigned long prevMessage = 0;


void setup() {
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

  Serial.begin(115200);
}

void loop() {
  current = millis();
  if (current - prev >= 100) {
    calculateWheelVel();
    calculatePID();
    drive();

    

    if (current - prevMessage >= 100) {
      calculateRobotVel();
      calculateRobotAngVel();
      calculateOdom();
      readPiSerial();
      robotValues["x"] = xRobot;
      robotValues["y"] = yRobot;
      robotValues["theta"] = theta;
      robotValues["liniarx"] = robotVel;
      robotValues["angularz"] = robotAngVel;
  
      serializeJson(robotValues, Serial);
      
      prevMessage = current;
    }
  }
}


void calculateWheelVel() {
  pulseCountLeft = encoderLeftPos - encoderLeftPosPrev;
  velocityLeft = pulseCountLeft * DistancePerPulse;
  velocityLeft = velocityLeft * 100;
  encoderLeftPosPrev = encoderLeftPos;

  pulseCountRight = encoderRightPos - encoderRightPosPrev;
  velocityRight = pulseCountRight * DistancePerPulse;
  velocityRight = velocityRight * 100;
  encoderRightPosPrev = encoderRightPos;
}

void readPiSerial() {
  if (Serial.available() > 0) {
    
    String result = Serial.readString();

    const int length = result.length();

    char* char_array = result.c_str();
    int commaplaced = 0;
    for (int i = 0; i < result.length(); i++) {
      if (strcmp(char_array[i], ',') == 0) {
        commaplaced = i;
      }
    }
    // piLiniar = Serial.readStringUntil('\n').toDouble();
    // piAngular = Serial.readStringUntil('\n').toDouble();

    piLiniar = result.substring(0, (commaplaced - 1)).toDouble();
    piAngular = result.substring((commaplaced + 1), (result.length() - 1)).toDouble();
    // piLiniar = 1.0;
    // piAngular = 0.0;
    piCalcLeft = piLiniar - ((piAngular * LengthBetweenWheels) / 2);
    piCalcRight = piLiniar + ((piAngular * LengthBetweenWheels) / 2);
    
  }
}

void calculatePID() {
  SetpointLeft = piCalcLeft;
  InputLeft = velocityLeft;
  PIDLeft.Compute();

  SetpointRight = piCalcRight;
  InputRight = velocityRight;
  PIDRight.Compute();
}

void calculateRobotVel() {
  robotVel = (velocityRight + velocityLeft) / 2;
}

void calculateRobotAngVel() {
  robotAngVel = (velocityRight - velocityLeft) / LengthBetweenWheels;
}

void calculateOdom() {

  if (robotAngVel != 0) {
    xRobot += robotVel / robotAngVel * (sin(robotAngVel * 0.01 + theta) - sin(theta));
    yRobot += (-1 * robotVel) / robotAngVel * (cos(robotAngVel * 0.01 + theta) - cos(theta));
  } else {
    xRobot += robotVel * cos(theta) * 0.01;
    yRobot += robotVel * sin(theta) * 0.01;
  }

  theta += robotAngVel * 0.01;

  if (theta > PI)
    theta = (-1 * PI) + (theta + (-1 * PI));

  if (theta < (-1 * PI))
    theta = PI + (theta + PI);
}


void drive() {
  if (piCalcLeft == 0 && velocityLeft == 0) {
    digitalWrite(LeftM1, LOW);
    digitalWrite(LeftM2, LOW);
    analogWrite(LeftPwm, 0);
  } else {
    if (OutputLeft > 0) {
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, HIGH);
      analogWrite(LeftPwm, abs(OutputLeft));
    } else if (OutputLeft < 0) {
      digitalWrite(LeftM1, HIGH);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, abs(OutputLeft));
    } else {
      digitalWrite(LeftM1, LOW);
      digitalWrite(LeftM2, LOW);
      analogWrite(LeftPwm, 0);
    }
  }
  if (piCalcRight == 0 && velocityRight == 0) {
    digitalWrite(RightM1, LOW);
    digitalWrite(RightM2, LOW);
    analogWrite(RightPwm, 0);
  } else {
    if (OutputRight > 0) {
      digitalWrite(RightM1, HIGH);
      digitalWrite(RightM2, LOW);
      analogWrite(RightPwm, abs(OutputRight));
    } else if (OutputRight < 0) {
      digitalWrite(RightM1, LOW);
      digitalWrite(RightM2, HIGH);
      analogWrite(RightPwm, abs(OutputRight));
    } else {
      digitalWrite(RightM1, LOW);
      digitalWrite(RightM2, LOW);
      analogWrite(RightPwm, 0);
    }
  }
}

void doEncoderLeftA() {
  if (digitalRead(EncoderLeftA) == HIGH) {
    if (digitalRead(EncoderLeftB) == LOW) {
      encoderLeftPos = encoderLeftPos + 1;
    } else {
      encoderLeftPos = encoderLeftPos - 1;
    }
  } else {
    if (digitalRead(EncoderLeftB) == HIGH) {
      encoderLeftPos = encoderLeftPos + 1;
    } else {
      encoderLeftPos = encoderLeftPos - 1;
    }
  }
}

void doEncoderLeftB() {
  if (digitalRead(EncoderLeftB) == HIGH) {
    if (digitalRead(EncoderLeftA) == HIGH) {
      encoderLeftPos = encoderLeftPos + 1;
    } else {
      encoderLeftPos = encoderLeftPos - 1;
    }
  } else {
    if (digitalRead(EncoderLeftA) == LOW) {
      encoderLeftPos = encoderLeftPos + 1;
    } else {
      encoderLeftPos = encoderLeftPos - 1;
    }
  }
}

void doEncoderRightA() {
  if (digitalRead(EncoderRightA) == HIGH) {
    if (digitalRead(EncoderRightB) == LOW) {
      encoderRightPos = encoderRightPos + 1;
    } else {
      encoderRightPos = encoderRightPos - 1;
    }
  } else {
    if (digitalRead(EncoderRightB) == HIGH) {
      encoderRightPos = encoderRightPos + 1;
    } else {
      encoderRightPos = encoderRightPos - 1;
    }
  }
}

void doEncoderRightB() {
  if (digitalRead(EncoderRightB) == HIGH) {
    if (digitalRead(EncoderRightA) == HIGH) {
      encoderRightPos = encoderRightPos + 1;
    } else {
      encoderRightPos = encoderRightPos - 1;
    }
  } else {
    if (digitalRead(EncoderRightA) == LOW) {
      encoderRightPos = encoderRightPos + 1;
    } else {
      encoderRightPos = encoderRightPos - 1;
    }
  }
}
