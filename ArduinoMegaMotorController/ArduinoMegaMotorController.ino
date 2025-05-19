#include <AccelStepper.h>
#include "Uart.h"

// Define stepper motor connections
#define ANGLE1_STEP_PIN 2
#define ANGLE1_DIR_PIN  3

#define MOTOR_INTERFACE_TYPE 1 // Driver interface type

// Create a new instance of the AccelStepper class
AccelStepper stepper(MOTOR_INTERFACE_TYPE, ANGLE1_STEP_PIN, ANGLE1_DIR_PIN);

// Constants for motor configuration
const int STEPS_PER_REVOLUTION = 16 * 200;  // Correct for NEMA 17 (1.8° per step)
const float DEGREES_PER_SECOND = 5 * 360.0;     // Set your desired speed in degrees per second
const int SPEED = (int)(DEGREES_PER_SECOND * STEPS_PER_REVOLUTION / 360.0);  // Calculate steps per second
const int ACCELERATION = SPEED * 5;         // Acceleration is 5 x speed for smooth operation

void setup() {
  // Configure stepper
  stepper.setMaxSpeed(SPEED);
  stepper.setAcceleration(ACCELERATION);

  Serial.begin(9600);
  Serial.println("Stepper Motor Controller Started");
  Serial.print("Speed set to: ");
  Serial.print(DEGREES_PER_SECOND);
  Serial.println(" degrees per second");
  Serial.print("Calculated steps per second: ");
  Serial.println(SPEED);

  initUART(); // Initialize Serial1 for UART
}

// Function to convert angle to steps
long angleToSteps(float angle) {
  return (long)(angle * STEPS_PER_REVOLUTION / 360.0);
}

// Function to convert steps to angle
float stepsToAngle(long steps) {
  return (float)(steps * 360.0 / STEPS_PER_REVOLUTION);
}

// Function to start a rotation (returns immediately)
void startRotation(AccelStepper &stepper, float angle) {
  long steps = angleToSteps(angle);
  stepper.move(steps);
}

// Function to check if a stepper is still moving
bool isMoving(AccelStepper &stepper) {
  return stepper.distanceToGo() != 0;
}

// Function to run one step of the motor movement
void runStepper(AccelStepper &stepper) {
  stepper.run();
}

// Function to rotate motor by specified angle and wait for completion
void rotateAngle(AccelStepper &stepper, float angle) {
  startRotation(stepper, angle);

  // Store initial position for relative angle calculation
  long startPos = stepper.currentPosition();
  unsigned long lastPrintTime = 0;
  const unsigned long PRINT_INTERVAL = 100; // Print every 100ms

  while(isMoving(stepper)) {
    runStepper(stepper);

    // Print current angle every PRINT_INTERVAL milliseconds
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      long currentPos = stepper.currentPosition();
      float relativeAngle = stepsToAngle(currentPos - startPos);
      float absoluteAngle = stepsToAngle(currentPos);

      Serial.print("Target: ");
      Serial.print(angle);
      Serial.print("°, Progress: ");
      Serial.print(relativeAngle);
      Serial.print("°, Absolute: ");
      Serial.print(absoluteAngle);
      Serial.println("°");

      lastPrintTime = currentTime;
    }
  }
}

void loop() {
  checkUART(); // Check for new UART messages and update jointAngles/gripperState

  // Print received values
  Serial.print("Angles: ");
  for (int i = 0; i < 5; ++i) {
    Serial.print(jointAngles[i]);
    Serial.print(i < 4 ? ", " : "");
  }
  Serial.print(" | Gripper: ");
  Serial.println(gripperState);

  // Move the stepper to the target angle (angle 1)
  static float lastAngle = 0;
  float targetAngle = jointAngles[0];
  if (targetAngle != lastAngle) {
    float delta = targetAngle - lastAngle;
    rotateAngle(stepper, delta);
    lastAngle = targetAngle;
  }

  delay(10);
}