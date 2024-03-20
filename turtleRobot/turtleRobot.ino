#define IN1 5
#define IN2 4
#define pwm 6
void setup()
{
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(pwm, OUTPUT);
Serial.begin(9600);
}
 
void loop()
{
// Rotate the Motor A clockwise
digitalWrite(pwm,HIGH);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
Serial.println("rotate moter A");




}
