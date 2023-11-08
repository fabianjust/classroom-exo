#include <Servo.h>
Servo myservo;        // create servo object to control a servo
int servo_control_pin = 5;
int pos = 0;

int emgAnalogInPin2 = A2; //EMG1 // Tricep
int emgSignal2 = 0;
int emgAnalogInPin1 = A1; //EMG0 // Beciep
int emgSignal1 = 0;

float Beciep = 0;
float Tricep = 0;

// Adjustable Thresholds
int Servo_Speed = 2;
float Beciep_Threshold = 700;
float Tricep_Threshold = 700;
//////////////////////////////

void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // LOW means Servo is OFF // HIGH
  delay(20);
  myservo.attach(servo_control_pin, 850, 2150);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);
}

void loop() {
  Beciep = EMG_B();
  Tricep = EMG_T();
  Serial.println(Beciep);
  Serial.print(",");
  Serial.println(Tricep);

  if (Beciep >= Beciep_Threshold && pos < 135) {
    pos = pos + Servo_Speed;
    myservo.write(180 - pos);
    delay(10);
  }
  else if (Tricep >= Tricep_Threshold && pos > 0) {
    pos = pos - Servo_Speed;
    myservo.write(180 - pos);
    delay(10);
  }
}

float EMG_B() {
  emgSignal1 = analogRead(emgAnalogInPin1);
  delay(10);
  return emgSignal1;
}

float EMG_T() {
  emgSignal2 = analogRead(emgAnalogInPin2);
  delay(10);
  return emgSignal2;
}
