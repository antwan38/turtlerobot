#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
 
void setup()
{
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
}
 
void loop()
{
// Rotate the Motor A clockwise
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
delay(2000);
// Motor A
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
delay(500);
// Rotate the Motor B clockwise
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
delay(2000);
// Motor B
digitalWrite(IN3, HIGH);
digitalWrite(IN4, HIGH);
delay(500);
}
