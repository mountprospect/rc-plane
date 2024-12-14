#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>  // Include the LIS3MDL library

// Create sensor objects
Adafruit_LSM6DS33 imu;
Adafruit_LIS3MDL mag;

// Define the onboard LED pin (ESP32 built-in LED)
const int ledPin = 13;  // Typically, the onboard LED on SparkFun Thing Plus is on GPIO 2

void setup() {
  // Start Serial communication
  Serial.begin(115200);
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
}

void loop() {
  // Turn on the LED at the start of the loop
  digitalWrite(ledPin, HIGH);

  // Create sensor event objects
  sensors_event_t accelEvent, gyroEvent, imuTemp;
  imu.getEvent(&accelEvent, &gyroEvent, &imuTemp);

  // Read magnetometer data (x, y, z)
  sensors_event_t magEvent;
  mag.getEvent(&magEvent);

  // Send accelerometer data to the Serial Plotter in the correct format
  Serial.print("Accel_X:");
  Serial.print(accelEvent.acceleration.x);
  Serial.print(",");
  Serial.print("Accel_Y:");
  Serial.print(accelEvent.acceleration.y);
  Serial.print(",");
  Serial.print("Accel_Z:");
  Serial.print(accelEvent.acceleration.z);
  Serial.print(",");

  Serial.print("Gyro_X:");
  Serial.print(gyroEvent.gyro.x);
  Serial.print(",");
  Serial.print("Gyro_Y:");
  Serial.print(gyroEvent.gyro.y);
  Serial.print(",");
  Serial.print("Gyro_Z:");
  Serial.print(gyroEvent.gyro.z);
  Serial.print(",");

  // Send magnetometer data to the Serial Plotter in the correct format
  
  Serial.print("Mag_X:");
  Serial.print(magEvent.magnetic.x);
  Serial.print(",");
  Serial.print("Mag_Y:");
  Serial.print(magEvent.magnetic.y);
  Serial.print(",");
  Serial.print("Mag_Z:");
  Serial.print(magEvent.magnetic.z);
  Serial.println(",");

  // End of the loop - Turn off the LED
  digitalWrite(ledPin, LOW);

  // Add a short delay before the next reading
  delay(20);  // 500 ms delay (adjust for your needs)
}
