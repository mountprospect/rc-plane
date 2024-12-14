//Created 12-13-2024

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>  // Include the LIS3MDL library

Servo rudderServo;

Adafruit_LSM6DS33 imu;
Adafruit_LIS3MDL mag;

const int ledPin = 13;

int ch4Pin = 2;  //pin plugged into ch4Pin on reciever. Default: 1492. Leftmost: 993 rightmost:1984.  rudder
int rudderServoPin = 4;

float ax, ay, az, rollRate, pitchRate, yawRate, magX, magY, magZ;

float yawThreshold = 0.5;          // Yaw rate threshold (degrees per second) to trigger rudder adjustment
float rudderCorrectionSpeed = 10;  // Rudder correction speed (adjust as needed)
int rudderNeutralPosition = 90;



void setup() {
  // Start Serial communication
  Serial.begin(115200);
  Serial.println("INIT...");
  delay(1000);  // Give time for the Serial monitor to start

  // Initialize onboard LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  // Turn on LED to indicate start

  // Initialize I2C communication (Qwiic uses I2C by default)
  Wire.begin();

  // Initialize the LSM6DS33 (Accelerometer + Gyroscope)
  if (!imu.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DS33 sensor!");
    while (1)
      ;  // Halt if initialization fails
  }
  Serial.println("LSM6DS33 sensor initialized.");

  // Initialize the LIS3MDL (Magnetometer)
  if (!mag.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL sensor!");
    while (1)
      ;  // Halt if initialization fails
  }
  Serial.println("LIS3MDL sensor initialized.");

  Serial.println("\n\nDEFLECTION TESTS...\n\n");

  Serial.println("RUDDER TEST...");
  rudderServo.attach(rudderServoPin);
  rudderServo.write(0);
  delay(750);
  rudderServo.write(180);
  delay(750);
  rudderServo.write(rudderNeutralPosition);
  delay(750);
  Serial.println("RUDDER TEST COMPLETE...");

  Serial.println("INIT COMPLETE...");
  delay(500);
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turn on LED to indicate start

  sensors_event_t accelEvent, gyroEvent, imuTemp;
  imu.getEvent(&accelEvent, &gyroEvent, &imuTemp);

  ax = accelEvent.acceleration.x;
  ay = accelEvent.acceleration.y;
  az = accelEvent.acceleration.z;

  //rollRate = gyroEvent.gyro.XX;
  //pitchRate = gyroEvent.gyro.XX;
  yawRate = gyroEvent.gyro.z;  // Rotation rate around the Z-axis (yaw)

  sensors_event_t magEvent;
  mag.getEvent(&magEvent);

  Serial.print("yawrate:");
  Serial.print(yawRate);
  Serial.println(",");


  long ch4PWM = pulseIn(ch4Pin, HIGH);  //THIS IS THE BOTTLENECK
  int manualRudder = map(ch4PWM, 1000, 2000, 0, 180);
  // int manualRudder = 5;
  // int ch4PWM = 1250;

  if (ch4PWM > 1000 && ch4PWM < 2000) {
    rudderServo.write(manualRudder);  // Override automation with manual input
  } else {
    // If the manual input is out of range (e.g., no signal), use automation

    // If the yaw rate exceeds the threshold, correct the yaw with the rudder
    if (abs(yawRate) > yawThreshold) {
      // If yaw rate is positive (turning right), steer left to correct
      if (yawRate > 0) {
        rudderServo.write(rudderNeutralPosition + rudderCorrectionSpeed);  // Adjust rudder to left
      }
      // If yaw rate is negative (turning left), steer right to correct
      else {
        rudderServo.write(rudderNeutralPosition - rudderCorrectionSpeed);  // Adjust rudder to right
      }
    }
    // If yaw rate is within the threshold, keep rudder centered
    else {
      rudderServo.write(rudderNeutralPosition);  // Keep rudder centered
    }
  }

  // Serial.print("Accel_X:");
  // Serial.print(accelEvent.acceleration.x);
  // Serial.print(",");
  // Serial.print("Accel_Y:");
  // Serial.print(accelEvent.acceleration.y);
  // Serial.print(",");
  // Serial.print("Accel_Z:");
  // Serial.print(accelEvent.acceleration.z);
  // Serial.print(",");

  // Serial.print("Gyro_X:");
  // Serial.print(gyroEvent.gyro.x);
  // Serial.print(",");
  // Serial.print("Gyro_Y:");
  // Serial.print(gyroEvent.gyro.y);
  // Serial.print(",");
  // Serial.print("Gyro_Z:");
  // Serial.print(gyroEvent.gyro.z);
  // Serial.print(",");

  // Serial.print("Mag_X:");
  // Serial.print(magEvent.magnetic.x);
  // Serial.print(",");
  // Serial.print("Mag_Y:");
  // Serial.print(magEvent.magnetic.y);
  // Serial.print(",");
  // Serial.print("Mag_Z:");
  // Serial.print(magEvent.magnetic.z);
  // Serial.println(",");

  // delay(20);
  digitalWrite(ledPin, LOW);
}
