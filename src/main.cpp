#include <Arduino.h>
#include <Encoder.h>

// Define pin variables
const int INA = 2;
const int INB = 4;
const int PWM_PIN = 3;

// Define maximum speed and commanded speed (manually update these while testing)
const int maxSpeed = 530; // Maximum speed in RPM
float commandedSpeed = 530.0; // Commanded speed in RPM
int commandedDirection = -1; // 1 for forward, -1 for reverse, 0 for stop

// Define variables for encoder reading and initialize object
const int encoderPinA = 5; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB = 6;
Encoder myEnc(encoderPinA, encoderPinB);

// Define values for converting the encoder readings to speed (RPM)
const int encoderCPR = 1200; // Counts per revolution of the encoder (output shaft)
unsigned long lastTime = 0;

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

  // Initialize pins for motor control
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  Serial.println("Motor Control Initialized");
}

long oldPosition  = 0;

void loop() {
  // Output commanded speed to the motor
  int pwmValue = speedToPWM(commandedSpeed);
  sendMotorOutput(pwmValue, commandedDirection);
  Serial.print("Commanded Speed (PWM): ");
  Serial.println(pwmValue);

  // Read the encoder position and calculate speed
  long newPosition = myEnc.read();
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) { // Update every 100 ms
    long deltaPosition = newPosition - oldPosition;
    float speedRPM = (deltaPosition / (float)encoderCPR) * (60000.0 / (currentTime - lastTime));
    Serial.print("Measured Speed (RPM): ");
    Serial.println(speedRPM);
    lastTime = currentTime;
    oldPosition = newPosition;
    delay(500);
  }
}