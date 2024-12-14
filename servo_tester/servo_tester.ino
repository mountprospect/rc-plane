//Created 12-12-24 to try to interface arduino with FS-iA6b reciever

#include <Servo.h>  // Include the Servo library


int ch4Pin = 2;  //pin plugged into ch4Pin on reciever. Default: 1492. Leftmost: 993 rightmost:1984.  rudder
int ch2Pin = 5;  //elevator
int rudderServoPin = 4;
int elevatorServoPin = 6;

Servo rudderServo;
Servo elevatorServo;

void setup() {

  Serial.begin(9600);
  Serial.println("INIT...");
  pinMode(13, OUTPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(ch2Pin, INPUT);

  delay(250);


  Serial.println("RUDDER TEST...");
  rudderServo.attach(rudderServoPin);

  rudderServo.write(0);
  delay(1500);
  rudderServo.write(180);
  delay(1500);
  rudderServo.write(90);
  delay(500);

  Serial.println("RUDDER TEST COMPLETE...");

  delay(500);

  Serial.println("ELEVATOR TEST...");
  elevatorServo.attach(elevatorServoPin);

  elevatorServo.write(180);
  delay(1500);
  elevatorServo.write(0);
  delay(1500);
  elevatorServo.write(110);
  delay(500);

  Serial.println("ELEVATOR TEST COMPLETE...");


  Serial.println("INIT COMPLETE...");
  delay(500);
}

void loop() {
  long ch4PWM = pulseIn(ch4Pin, HIGH);
  Serial.print("ch4: ");
  Serial.println(ch4PWM);

  int rudderPosition = map(ch4PWM, 1000, 2000, 0, 180);
  rudderServo.write(rudderPosition);

  long ch2PWM = pulseIn(ch2Pin, HIGH);
  Serial.print("ch2: ");
  Serial.println(ch2PWM);
  int elevatorPosition = map(ch2PWM, 1000, 2000, 0, 180);
  elevatorServo.write(elevatorPosition);


  delay(20);
}
