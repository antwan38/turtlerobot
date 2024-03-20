#define LeftM1 5
#define LeftM2 6
#define RightM1 7
#define RightM2 8
#define LeftPwm 3
#define RightPwm 9
#define EncoderLeftA 11
#define EncoderLeftB 12
#define EncoderRightA 1
#define EncoderRightB 2

int aLastStateLeft;
int aLastStateRight;
int counterLeft = 0;
int counterRight = 0; 
int aStateLeft;
int aStateRight;
  
void setup()
{
  pinMode(LeftM1, OUTPUT);
  pinMode(LeftM2, OUTPUT);
  pinMode(RightM1, OUTPUT);
  pinMode(RightM2, OUTPUT);
  pinMode(LeftPwm, OUTPUT);
  pinMode(RightPwm, OUTPUT);
  pinMode(EncoderLeftA, INPUT);
  pinMode(EncoderLeftB, INPUT);
  pinMode(EncoderRightA, INPUT);
  pinMode(EncoderRightB, INPUT);
  Serial.begin(115200);
}
 
void loop()
{
// Rotate the Motor A clockwise
while(counterLeft < 90){
 analogWrite(LeftPwm,50);
digitalWrite(LeftM1, HIGH);
digitalWrite(LeftM2, LOW);
ReadRotations();
 } 
 digitalWrite(LeftM1, HIGH);
digitalWrite(LeftM2, HIGH);

}

void ReadRotations(){


  aStateLeft = digitalRead(EncoderLeftA);
   if (aStateLeft != aLastStateLeft){     
     if (digitalRead(EncoderLeftB) != aStateLeft) { 
       counterLeft ++;
     } else {
       counterLeft --;
     }
     Serial.print("Positionleft: ");
     Serial.println(counterLeft);
   } 
   aLastStateLeft = aStateLeft;

   
  aStateRight = digitalRead(EncoderRightA);
   if (aStateRight != aLastStateRight){     
     if (digitalRead(EncoderRightB) != aStateRight) { 
       counterRight ++;
     } else {
       counterRight --;
     }
     Serial.print("PositionRight: ");
     Serial.println(counterRight);
   } 
   aLastStateRight = aStateRight;
  

   
}
