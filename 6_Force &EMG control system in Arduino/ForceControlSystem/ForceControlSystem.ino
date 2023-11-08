#include <Servo.h>
Servo myservo;        // create servo object to control a servo
int servo_control_pin = 5;
int pos = 0;          // variable to store the servo position

int forceAnalogInPin = A0;

float xn = 0;
float filtered;
float xn1 = 0;
float yn1 = 0;

int i = 0;
int Main = 0;
int current_angle = 0;

//Adjustable Value
int Force_offset = 365;
int Servo_Speed = 5;              // recommanded range between 1~10 . Not  more than 100 not less than zero
float Upper_Threshold = 20;         // recommanded range between 20~100.
float Lower_Threshold = -20;         // recommanded range between -20~-100.


void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // LOW means Servo is OFF // HIGH is ON
  delay(20);
  myservo.attach(servo_control_pin, 850, 2150);  
  myservo.write(180);
}

void loop() {
  filtered = forcesensor();                     // function to get the values form the force sensor and plot it.
  if (filtered >= Upper_Threshold && pos < 135) {
    pos = pos +Servo_Speed;
    myservo.write(180 - pos);
    delay(5);
  }
  else if (filtered < Lower_Threshold && pos > 0) {
    pos = pos - Servo_Speed;
    myservo.write(180 - pos);
    delay(5);
  }
}


float forcesensor() {
    xn = analogRead(forceAnalogInPin) - 385;
    filtered = 0.969 * yn1 + 0.0155 * xn + 0.0155 * xn1;
    xn1 = xn;
    yn1 = filtered;
    Serial.println(xn);
    Serial.print(",");
    Serial.println(filtered);
    delay(10);
    return filtered;
}
