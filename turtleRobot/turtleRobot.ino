#define LeftM1 5
#define LeftM2 6
#define RightM1 7
#define RightM2 8
#define LeftPwm 4
#define RightPwm 9
#define EncoderLeftA 11
#define EncoderLeftB 12
#define EncoderRightA 1
#define EncoderRightB 2

int aLastStateLeft;
int aLastStateRight;
int counterLeft = 0;
int counterRight = 0; 
  
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
  Serial.begin(9600);
}
 
void loop()
{
// Rotate the Motor A clockwise
analogWrite(LeftPwm,255);
digitalWrite(LeftM1, HIGH);
digitalWrite(LeftM2, LOW);
ReadRotations();
}

void ReadRotations(){

  int aState;
  aState = digitalRead(EncoderLeftA);
   if (aState != aLastStateLeft){     
     if (digitalRead(EncoderLeftB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     Serial.print("PositionRight: ");
     Serial.println(counter);
   } 
   aLastStateLeft = aState;

  aState = digitalRead(EncoderRighttA);
   if (aState != aLastStateRight){     
     if (digitalRead(EncoderRightB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     Serial.print("PositionRight: ");
     Serial.println(counter);
   } 
   aLastStateRight = aState;

   
}
