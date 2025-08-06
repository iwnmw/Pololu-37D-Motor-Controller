#include <Arduino.h>
#include <Encoder.h>
#include "PIDController.h"

// Define pin variables
const int INA = 2;
const int INB = 4;
const int PWM_PIN = 3;

// Define maximum speed and commanded speed (manually update these while testing)
const int maxSpeed = 530; // Maximum speed in RPM
float RPMTarget = 330.0; // Commanded speed in RPM
int commandedDirection = -1; // 1 for forward, -1 for reverse, 0 for stop

// Define variables for encoder reading and initialize object
const int encoderPinA = 5; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB = 6;
Encoder myEnc(encoderPinA, encoderPinB);

// Define values for converting the encoder readings to speed (RPM)
const int encoderCPR = 1200; // Counts per revolution of the encoder (output shaft)
unsigned long lastTime = 0;

// Initialize the PID controller object
// Note: PID controller controls rate of change of PWM signal, not speed directly, which is why the maximum and minimum outputs are set as they are
PIDController PID(1.0, 0.1, 0.05, -100, 100); // PID gains and output limits

// Define a function to convert from rotational speed to PWM output
int speedToPWM(int commandedSpeed) {
  // Assuming speed is in RPM and we want to convert it to a PWM value
  return map(commandedSpeed, 0, maxSpeed, 0, 255);
}

// Define a function to send a motor output
void sendMotorOutput(int speed, int direction) {

  if (direction == 1) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else if (direction == -1) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
  }
  analogWrite(PWM_PIN, abs(speed));
}

void setup() {
  Serial.begin(115200);

  // Initialize pins for motor control
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  Serial.println("Motor Control Initialized");
}

long oldPosition  = 0;

void loop() {
  // Read the encoder position and calculate speed
  long newPosition = myEnc.read();
  unsigned long currentTime = millis();
  float dt = currentTime - lastTime; // Time difference in milliseconds
  long deltaPosition = newPosition - oldPosition;
  float RPMActual = (deltaPosition / (float)encoderCPR) * (60000.0 / dt);
  lastTime = currentTime;
  oldPosition = newPosition;

  // Compute the baselinePWM value based on commanded speed
  int basePWM = speedToPWM(RPMTarget);

  // Use the PID Controller to create a PWM adjustment
  int deltaPWM = PID.compute(RPMTarget, RPMActual, dt*1000); // Convert dt to seconds for PID computation
  int totalPWM = constrain(basePWM + deltaPWM, 0, 255); // Ensure PWM is within valid range
  sendMotorOutput(totalPWM, commandedDirection);

  // Print the current state
  Serial.print("setpoint:");
  Serial.print(RPMTarget);
  Serial.print(" rpm:");
  Serial.print(RPMActual);
  Serial.print(" pwm:");
  Serial.print(totalPWM);
  Serial.print(" deltaPWM:");
  Serial.println(deltaPWM);

  //delay(50);
}