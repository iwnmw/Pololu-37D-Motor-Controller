#include <Arduino.h>

// Define pin variables
const int INA = 2;
const int INB = 4;
const int PWM_PIN = 3;

// Define maximum speed and commanded speed (manually update these while testing)
const int maxSpeed = 530; // Maximum speed in RPM
float commandedSpeed = 530.0; // Commanded speed in RPM
int commandedDirection = -1; // 1 for forward, -1 for reverse, 0 for stop

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
  Serial.begin(9600);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  Serial.println("Motor Control Initialized");
}

void loop() {
  // Output commanded speed to the motor
  int pwmValue = speedToPWM(commandedSpeed);
  sendMotorOutput(pwmValue, commandedDirection);
}