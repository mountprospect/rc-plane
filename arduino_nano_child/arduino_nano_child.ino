//tester created 12-15-2024

#include <Wire.h>
#include <Servo.h>

Servo rudderServo;
const int rudderPin = 12;  // Pin connected to the servo signal
int ch4Pin = 11;           // Pin for PWM signal from the receiver

long ch4PWM = 0;           // Variable to store PWM signal from the receiver
int rudderPosition = 90;   // Default rudder position for manual inputs (neutral)
int autoRudderPosition = 90; // Default rudder position for automated inputs (from ESP32)

int rudderDeadband = 50;   // Deadband to filter out small fluctuations in PWM
int minPWM = 985;          // Minimum PWM value for the receiver (adjust as needed)
int maxPWM = 2000;         // Maximum PWM value for the receiver (adjust as needed)
int neutralPWM = 1490;     // Typical neutral PWM value for the receiver
int pwmChangeThreshold = 100;  // Lower threshold to detect large jumps (e.g., 1490 to 1113)

void setup() {
  Serial.begin(115200);
  Wire.begin(8);                // Set Arduino Nano I2C address to 8 (same as ESP32 master)
  Wire.onReceive(receiveData);  // Set the function to be called when data is received

  rudderServo.attach(rudderPin);      // Attach the servo to the specified pin
  rudderServo.write(rudderPosition);  // Start with neutral position (90 degrees)

  pinMode(ch4Pin, INPUT);  // Set the PWM input pin
}

void loop() {
  // Read PWM signal on ch4Pin from receiver
  ch4PWM = pulseIn(ch4Pin, HIGH);  // Read the pulse width of the PWM signal
  Serial.println(ch4PWM);

  // If the PWM signal is outside the deadband (indicating manual input)
  if (ch4PWM < neutralPWM - rudderDeadband || ch4PWM > neutralPWM + rudderDeadband) {
    // Manual input is present, we can map the PWM directly to a rudder position
    int manualRudderPosition = map(ch4PWM, minPWM, maxPWM, 0, 180);  // Map PWM signal to rudder position
    rudderPosition = manualRudderPosition;

    Serial.print("Manual Rudder: ");
    Serial.println(rudderPosition);
  }
  else {
    // If the PWM signal is within the deadband range (neutral), use automated correction received via I2C
    Serial.print("Neutral PWM - Automated Rudder: ");
    Serial.println(autoRudderPosition);  // Update with the value received from ESP32

    // If the yaw rate correction from the ESP32 is valid, adjust the rudder position
    rudderPosition = autoRudderPosition;
  }

  // Move the servo to the desired position (either manual or automated)
  rudderServo.write(rudderPosition);

  delay(100);  // Short delay for stability
}

// Function to handle incoming I2C data (from ESP32)
void receiveData(int byteCount) {
  while (Wire.available()) {
    autoRudderPosition = Wire.read();  // Read the rudder position sent by ESP32
    // Ensure that the rudder position is within a valid range
    if (autoRudderPosition < 0) autoRudderPosition = 0;
    if (autoRudderPosition > 180) autoRudderPosition = 180;
    Serial.print("Received from ESP32: ");
    Serial.println(autoRudderPosition);  // Debug: print received position
  }
}

