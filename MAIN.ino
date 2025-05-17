#include <NewPing.h>

// Pin Definitions
#define TRIGGER_PINL  A2  // Left sensor trigger
#define ECHO_PINL     A3  // Left sensor echo

#define TRIGGER_PINF  A0  // Front sensor trigger
#define ECHO_PINF     A1  // FIXED: Was incorrectly set to A2 (same as left trigger)

#define TRIGGER_PINR  A4  // Right sensor trigger
#define ECHO_PINR     A5  // Right sensor echo

#define MAX_DISTANCE 90  // Maximum distance to ping (in centimeters)

// Direction constants
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

// Motor pins
int en1 = 4;  // Left motor input 1
int en2 = 5;  // Left motor input 2
int en3 = 6;  // Right motor input 1
int en4 = 7;  // Right motor input 2
int enA = 9;  // Left motor enable (PWM)
int enB = 10; // Right motor enable (PWM)

// PID Control Parameters - ADJUSTED for smoother control
float P = 0.8;    // Increased from 0.7 for more responsive correction
float D = 0.6;    // Increased from 0.5 for better stability
float I = 0.3;    // Decreased from 0.4 to reduce oscillation
float oldErrorP = 0;
float totalError = 0;
int offset = 5;   // Offset for wall following

// Wall detection thresholds - ADJUSTED to avoid collisions
int wall_threshold = 15;   // Increased from 13
int front_threshold = 15;  // Increased from 7 to detect walls earlier

// Speed settings
int baseSpeed = 70;  // Base motor speed
int turnSpeed = 90;  // Speed during turns
int RMS;  // Right motor speed
int LMS;  // Left motor speed

// State variables
boolean frontwall = false;
boolean leftwall = false;
boolean rightwall = false;
boolean first_turn = false;
boolean rightWallFollow = false;
boolean leftWallFollow = false;

// Setup sonar sensors
NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

// Sensor readings
float oldLeftSensor = 0, oldRightSensor = 0, leftSensor = 0, rightSensor = 0;
float frontSensor = 0, oldFrontSensor = 0, lSensor = 0, rSensor = 0, fSensor = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize all motor pins as outputs
  for (int i = 2; i <= 13; i++)
    pinMode(i, OUTPUT);
    
  // Set initial state
  first_turn = false;
  rightWallFollow = false;
  leftWallFollow = false;
  
  // Initial stop
  setDirection(STOP);
  delay(1000);
}

void loop() {
  // Read sensor values
  ReadSensors();
  
  // Detect walls
  walls();
  
  // Debug output
  printSensorReadings();
  
  // Safety check - stop if sensors return invalid readings
  if ((leftSensor == 0 || leftSensor > 100) && 
      (rightSensor == 0 || rightSensor > 100) && 
      (frontSensor == 0 || frontSensor > 100)) {
    setDirection(STOP);
    delay(100);
    return;
  }
  
  // Emergency stop if front distance is very small
  if (frontSensor > 0 && frontSensor < 5) {
    setDirection(STOP);
    delay(100);
    setDirection(BACKWARD);
    delay(300);
    return;
  }
  
  // Wall following logic
  if (!first_turn) {
    // Initial mode - find a wall to follow
    if (frontwall) {
      // Front blocked, decide which way to turn
      if (!leftwall && !rightwall) {
        // If both sides are open, prefer left
        executeLeftTurn();
        first_turn = true;
        leftWallFollow = true;
      } else if (!leftwall) {
        executeLeftTurn();
        first_turn = true;
        leftWallFollow = true;
      } else if (!rightwall) {
        executeRightTurn();
        first_turn = true;
        rightWallFollow = true;
      } else {
        // All sides blocked, back up and try again
        setDirection(BACKWARD);
        analogWrite(enA, baseSpeed);
        analogWrite(enB, baseSpeed);
        delay(500);
      }
    } else {
      // No front wall, use PID to go straight
      pid_start();
    }
  } else {
    // Handle the case where we're already following a wall
    if (frontwall) {
      // Front wall detected, turn according to which wall we're following
      if (leftWallFollow) {
        if (!rightwall) {
          executeRightTurn();
        } else {
          executeLeftTurn();
        }
      } else if (rightWallFollow) {
        if (!leftwall) {
          executeLeftTurn();
        } else {
          executeRightTurn();
        }
      }
    } else {
      // No front wall - continue wall following
      if (leftWallFollow) {
        PID(true);
      } else if (rightWallFollow) {
        PID(false);
      }
    }
  }
  
  // Brief delay for stability
  delay(50);
}

// Execute a sharper left turn to avoid walls
void executeLeftTurn() {
  setDirection(STOP);
  delay(100);
  setDirection(LEFT);
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
  delay(400);  // Longer turn duration
  setDirection(STOP);
  delay(100);
}

// Execute a sharper right turn to avoid walls
void executeRightTurn() {
  setDirection(STOP);
  delay(100);
  setDirection(RIGHT);
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
  delay(400);  // Longer turn duration
  setDirection(STOP);
  delay(100);
}

// Print sensor readings for debugging
void printSensorReadings() {
  Serial.print(" Left: ");
  Serial.print(leftSensor);
  Serial.print(" cm | Right: ");
  Serial.print(rightSensor);
  Serial.print(" cm | Front: ");
  Serial.print(frontSensor);
  Serial.print(" cm | Error: ");
  Serial.println(totalError);
}

// Motor direction control
void setDirection(int dir) {
  switch (dir) {
    case FORWARD:
      digitalWrite(en1, HIGH);  // Left wheel forward
      digitalWrite(en2, LOW);
      digitalWrite(en3, HIGH);  // Right wheel forward
      digitalWrite(en4, LOW);
      break;
    case LEFT:
      digitalWrite(en1, LOW);   // Left wheel backward
      digitalWrite(en2, HIGH);
      digitalWrite(en3, HIGH);  // Right wheel forward
      digitalWrite(en4, LOW);
      break;
    case RIGHT:
      digitalWrite(en1, HIGH);  // Left wheel forward
      digitalWrite(en2, LOW);
      digitalWrite(en3, LOW);   // Right wheel backward
      digitalWrite(en4, HIGH);
      break;
    case STOP:
      digitalWrite(en1, LOW);   // Both wheels stop
      digitalWrite(en2, LOW);
      digitalWrite(en3, LOW);
      digitalWrite(en4, LOW);
      break;
    case BACKWARD:
      digitalWrite(en1, LOW);   // Both wheels backward
      digitalWrite(en2, HIGH);
      digitalWrite(en3, LOW);
      digitalWrite(en4, HIGH);
      break;
  }
}

// Read and smooth sensor values
void ReadSensors() {
  // Get raw readings
  lSensor = sonarLeft.ping_cm();
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();
  
  // Apply filtering to reduce noise
  if (lSensor > 0) {
    leftSensor = (lSensor + oldLeftSensor) / 2;
    oldLeftSensor = leftSensor;
  }
  
  if (rSensor > 0) {
    rightSensor = (rSensor + oldRightSensor) / 2;
    oldRightSensor = rightSensor;
  }
  
  if (fSensor > 0) {
    frontSensor = (fSensor + oldFrontSensor) / 2;
    oldFrontSensor = frontSensor;
  }
}

// Initial PID control for going straight
void pid_start() {
  // Calculate PID error
  float errorP = leftSensor - rightSensor;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * totalError + errorP;  // FIXED: was using errorI variable that wasn't updated
  
  totalError = P * errorP + D * errorD + I * errorI;
  oldErrorP = errorP;
  
  // Calculate motor speeds
  RMS = baseSpeed + totalError;
  LMS = baseSpeed - totalError;
  
  // Ensure motor speeds are valid
  RMS = constrain(RMS, -255, 255);
  LMS = constrain(LMS, -255, 255);
  
  // Apply motor direction based on speed values
  if (RMS < 0) {
    RMS = map(RMS, 0, -255, 0, 255);
    analogWrite(enA, RMS);
    analogWrite(enB, LMS);
    setDirection(RIGHT);
  }
  else if (LMS < 0) {
    LMS = map(LMS, 0, -255, 0, 255);
    analogWrite(enA, RMS);
    analogWrite(enB, LMS);
    setDirection(LEFT);
  }
  else {
    analogWrite(enA, RMS);
    analogWrite(enB, LMS);
    setDirection(FORWARD);
  }
}

// PID control for wall following
void PID(boolean left) {
  float errorP;
  
  // Calculate error based on which wall we're following
  if (left) {
    errorP = leftSensor - wall_threshold - offset;
  } else {
    errorP = wall_threshold - rightSensor - offset;
  }
  
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * totalError + errorP;  // FIXED: was using errorI variable incorrectly
  
  totalError = P * errorP + D * errorD + I * errorI;
  oldErrorP = errorP;
  
  // Calculate motor speeds
  RMS = baseSpeed + totalError;
  LMS = baseSpeed - totalError;
  
  // Ensure motor speeds are valid
  RMS = constrain(RMS, -255, 255);
  LMS = constrain(LMS, -255, 255);
  
  // Apply motor direction based on speed values
  if (RMS < 0) {
    RMS = map(RMS, 0, -255, 0, 255);
    analogWrite(enA, RMS);
    analogWrite(enB, LMS);
    setDirection(RIGHT);
  }
  else if (LMS < 0) {
    LMS = map(LMS, 0, -255, 0, 255);
    analogWrite(enA, RMS);
    analogWrite(enB, LMS);
    setDirection(LEFT);
  }
  else {
    analogWrite(enA, RMS);
    analogWrite(enB, LMS);
    setDirection(FORWARD);
  }
}

// Detect walls based on sensor readings and thresholds
void walls() {
  leftwall = (leftSensor > 0 && leftSensor < wall_threshold);
  rightwall = (rightSensor > 0 && rightSensor < wall_threshold);
  frontwall = (frontSensor > 0 && frontSensor < front_threshold);
}
