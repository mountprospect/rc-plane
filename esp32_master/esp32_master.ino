//tester created 12-15-2024

#include <Wire.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>  // Include the LIS3MDL library

const int ledPin = 13;  // Onboard LED pin
const int i2cAddress = 8;  // I2C address for Arduino Nano (slave)

Adafruit_LSM6DS33 imu;
Adafruit_LIS3MDL mag;

float yawRate = 0;            // Yaw rate from IMU
float yawRateFiltered = 0;    // Filtered yaw rate
float yawThreshold = 0.05;    // Threshold for yaw correction (increase for less sensitive)
int rudderNeutralPosition = 90;    // Neutral rudder position (centered)
float maxRudderDeflection = 45;   // Maximum rudder deflection in degrees (increase for bigger movement)
float proportionalGain = 3;     // Proportional gain (increase for more aggressive correction)
int rudderPosition = rudderNeutralPosition;  // Initial rudder position

// Initialize IMU and magnetometer
void setup() {
  Serial.begin(115200);
  Serial.println("INIT...");
  delay(1000);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  Wire.begin();

  if (!imu.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DS33 sensor!");
    while (1);
  }
  Serial.println("LSM6DS33 sensor initialized.");

  if (!mag.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL sensor!");
    while (1);
  }
  Serial.println("LIS3MDL sensor initialized.");
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turn on LED to indicate start

  // Read sensor values
  sensors_event_t accelEvent, gyroEvent, imuTemp;
  imu.getEvent(&accelEvent, &gyroEvent, &imuTemp);

  // Get yaw rate (rotation around Z-axis) from the gyroscope
  yawRate = gyroEvent.gyro.z;

  // Apply a simple low-pass filter to smooth out the yaw rate (for smoother response)
  yawRateFiltered = 0.9 * yawRateFiltered + 0.1 * yawRate;  // 90% previous value + 10% current value

  Serial.print("Yaw rate: ");
  Serial.println(yawRateFiltered);

  // Apply proportional control based on the yaw rate
  if (abs(yawRateFiltered) > yawThreshold) {
    // Calculate rudder deflection proportional to yaw rate
    float rudderCorrection = proportionalGain * yawRateFiltered;
    
    // Clamp rudder correction to be within the allowable deflection range
    rudderCorrection = constrain(rudderCorrection, -maxRudderDeflection, maxRudderDeflection);

    // Adjust rudder position
    rudderPosition = rudderNeutralPosition - rudderCorrection;

    // Ensure the rudder position stays within valid bounds (0 to 180 degrees)
    rudderPosition = constrain(rudderPosition, 0, 180);
    sendCorrectionToArduino(rudderPosition);  // Send updated rudder position to Arduino
  } else {
    // If yaw rate is within threshold, keep rudder centered
    sendCorrectionToArduino(rudderNeutralPosition);  // Keep rudder centered
  }

  delay(50);  // Update rate (adjust as necessary)
  digitalWrite(ledPin, LOW);  // Turn off LED
}

// Function to send rudder position to Arduino via I2C
void sendCorrectionToArduino(int rudderPosition) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(rudderPosition);
  Wire.endTransmission();
  Serial.print("Sending rudder position: ");
  Serial.println(rudderPosition);
}

