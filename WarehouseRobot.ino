/*
 * Warehouse Robot - Autonomous Navigation and Transport System
 * 
 * This robot uses:
 * - Ultrasonic sensor (HC-SR04) for obstacle detection
 * - IR line tracking sensors for path following
 * - DC Motors with L298N motor driver for movement
 * - Servo motors for robotic arm control
 * 
 * Created for autonomous warehouse navigation and goods transportation
 */

// ==================== PIN DEFINITIONS ====================

// Motor Driver (L298N) Pins
#define MOTOR_LEFT_ENA 5    // Enable pin for left motor (PWM)
#define MOTOR_LEFT_IN1 7    // Input 1 for left motor
#define MOTOR_LEFT_IN2 8    // Input 2 for left motor
#define MOTOR_RIGHT_ENB 6   // Enable pin for right motor (PWM)
#define MOTOR_RIGHT_IN3 9   // Input 1 for right motor
#define MOTOR_RIGHT_IN4 10  // Input 2 for right motor

// Ultrasonic Sensor (HC-SR04) Pins
#define TRIGGER_PIN 12
#define ECHO_PIN 13

// IR Line Tracking Sensor Pins (using 5 sensors for better line following)
// Note: Adjust based on your sensor configuration (some sensors may use analog pins)
#define IR_SENSOR_LEFT 2    // Leftmost sensor
#define IR_SENSOR_LEFT_CENTER 3
#define IR_SENSOR_CENTER 4
#define IR_SENSOR_RIGHT_CENTER 11
#define IR_SENSOR_RIGHT 14  // Rightmost sensor (digital pin 14, which is A0)

// Robotic Arm Servo Pins
// Using analog pins A1-A5 (these can be used as digital pins 15-19)
#define SERVO_BASE A1       // Base rotation
#define SERVO_SHOULDER A2   // Shoulder joint
#define SERVO_ELBOW A3      // Elbow joint
#define SERVO_WRIST A4      // Wrist joint
#define SERVO_GRIPPER A5    // Gripper

// ==================== LIBRARIES ====================
#include <Servo.h>

// ==================== GLOBAL VARIABLES ====================

// Servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

// Motor speed control
int motorSpeed = 150;        // Base speed (0-255)
int turnSpeed = 100;         // Speed for turning
int maxSpeed = 200;          // Maximum speed

// Obstacle detection
int obstacleDistance = 0;
int obstacleThreshold = 15;  // Distance in cm to trigger obstacle avoidance
bool obstacleDetected = false;

// Line tracking sensor states
bool irLeft = false;
bool irLeftCenter = false;
bool irCenter = false;
bool irRightCenter = false;
bool irRight = false;

// Robot states
enum RobotState {
  LINE_FOLLOWING,
  OBSTACLE_AVOIDANCE,
  PICKUP_ITEM,
  DROPOFF_ITEM,
  IDLE
};

RobotState currentState = LINE_FOLLOWING;

// Arm positions
int baseAngle = 90;
int shoulderAngle = 90;
int elbowAngle = 90;
int wristAngle = 90;
int gripperAngle = 0;  // 0 = closed, 180 = open

// ==================== SETUP ====================
void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);
  Serial.println("Warehouse Robot Initializing...");

  // Configure motor pins
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);

  // Configure ultrasonic sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configure IR sensor pins
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_LEFT_CENTER, INPUT);
  pinMode(IR_SENSOR_CENTER, INPUT);
  pinMode(IR_SENSOR_RIGHT_CENTER, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  // Attach servos
  baseServo.attach(SERVO_BASE);
  shoulderServo.attach(SERVO_SHOULDER);
  elbowServo.attach(SERVO_ELBOW);
  wristServo.attach(SERVO_WRIST);
  gripperServo.attach(SERVO_GRIPPER);

  // Initialize arm to default position
  initializeArm();

  // Stop motors initially
  stopMotors();

  Serial.println("Warehouse Robot Ready!");
  delay(1000);
}

// ==================== MAIN LOOP ====================
void loop() {
  // Read sensors
  readIRSensors();
  obstacleDistance = readUltrasonicSensor();
  obstacleDetected = (obstacleDistance < obstacleThreshold && obstacleDistance > 0);

  // State machine
  switch (currentState) {
    case LINE_FOLLOWING:
      if (obstacleDetected) {
        currentState = OBSTACLE_AVOIDANCE;
        Serial.println("Obstacle detected! Switching to avoidance mode.");
      } else {
        followLine();
      }
      break;

    case OBSTACLE_AVOIDANCE:
      if (!obstacleDetected) {
        currentState = LINE_FOLLOWING;
        Serial.println("Obstacle cleared. Resuming line following.");
      } else {
        avoidObstacle();
      }
      break;

    case PICKUP_ITEM:
      pickupSequence();
      break;

    case DROPOFF_ITEM:
      dropoffSequence();
      break;

    case IDLE:
      stopMotors();
      break;
  }

  // Small delay for stability
  delay(10);
}

// ==================== MOTOR CONTROL FUNCTIONS ====================

void stopMotors() {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_LEFT_ENA, 0);
  analogWrite(MOTOR_RIGHT_ENB, 0);
}

void moveForward(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_LEFT_ENA, speed);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_LEFT_ENA, speed);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_LEFT_ENA, speed);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_LEFT_ENA, speed);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void pivotLeft(int speed) {
  // Left motor backward, right motor forward (sharp left turn)
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_LEFT_ENA, speed);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void pivotRight(int speed) {
  // Left motor forward, right motor backward (sharp right turn)
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_LEFT_ENA, speed);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

// ==================== ULTRASONIC SENSOR FUNCTIONS ====================

int readUltrasonicSensor() {
  // Clear trigger pin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // Send 10 microsecond pulse to trigger
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read echo pin and calculate distance
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;  // Convert to cm (speed of sound = 343 m/s)

  return distance;
}

// ==================== IR LINE TRACKING FUNCTIONS ====================

void readIRSensors() {
  // IR sensors typically return LOW when detecting a line (black) and HIGH when not
  // Adjust logic based on your sensor type (some work in reverse)
  irLeft = !digitalRead(IR_SENSOR_LEFT);
  irLeftCenter = !digitalRead(IR_SENSOR_LEFT_CENTER);
  irCenter = !digitalRead(IR_SENSOR_CENTER);
  irRightCenter = !digitalRead(IR_SENSOR_RIGHT_CENTER);
  irRight = !digitalRead(IR_SENSOR_RIGHT);
}

void followLine() {
  // Line following algorithm using 5 IR sensors
  // Priority-based line following for smooth navigation

  if (irCenter) {
    // Line is centered - move straight
    moveForward(motorSpeed);
  }
  else if (irLeftCenter && !irRightCenter) {
    // Line slightly to the left - slight left adjustment
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN3, HIGH);
    digitalWrite(MOTOR_RIGHT_IN4, LOW);
    analogWrite(MOTOR_LEFT_ENA, motorSpeed - 30);
    analogWrite(MOTOR_RIGHT_ENB, motorSpeed);
  }
  else if (irRightCenter && !irLeftCenter) {
    // Line slightly to the right - slight right adjustment
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN3, HIGH);
    digitalWrite(MOTOR_RIGHT_IN4, LOW);
    analogWrite(MOTOR_LEFT_ENA, motorSpeed);
    analogWrite(MOTOR_RIGHT_ENB, motorSpeed - 30);
  }
  else if (irLeft && !irRight) {
    // Line far to the left - sharp left turn
    turnLeft(turnSpeed);
  }
  else if (irRight && !irLeft) {
    // Line far to the right - sharp right turn
    turnRight(turnSpeed);
  }
  else if (irLeft && irLeftCenter) {
    // Strong left detection
    pivotLeft(turnSpeed);
  }
  else if (irRight && irRightCenter) {
    // Strong right detection
    pivotRight(turnSpeed);
  }
  else {
    // No line detected - continue last known direction or search
    // In a real scenario, you might want to slow down and search
    moveForward(motorSpeed / 2);
  }
}

// ==================== OBSTACLE AVOIDANCE ====================

void avoidObstacle() {
  // Stop and assess the situation
  stopMotors();
  delay(100);

  // Move backward slightly
  moveBackward(motorSpeed / 2);
  delay(300);
  stopMotors();
  delay(100);

  // Turn right to avoid obstacle
  turnRight(turnSpeed);
  delay(500);

  // Move forward while checking distance
  int attempts = 0;
  while (obstacleDetected && attempts < 10) {
    obstacleDistance = readUltrasonicSensor();
    obstacleDetected = (obstacleDistance < obstacleThreshold && obstacleDistance > 0);
    
    if (!obstacleDetected) {
      break;
    }
    
    moveForward(motorSpeed / 2);
    delay(100);
    attempts++;
  }

  // Try to find the line again
  stopMotors();
  delay(200);
  
  // Turn left to rejoin the line
  turnLeft(turnSpeed);
  delay(400);
}

// ==================== ROBOTIC ARM FUNCTIONS ====================

void initializeArm() {
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  wristServo.write(90);
  gripperServo.write(0);  // Gripper closed
  delay(500);
}

void openGripper() {
  for (int angle = gripperAngle; angle <= 180; angle += 5) {
    gripperServo.write(angle);
    delay(20);
  }
  gripperAngle = 180;
}

void closeGripper() {
  for (int angle = gripperAngle; angle >= 0; angle -= 5) {
    gripperServo.write(angle);
    delay(20);
  }
  gripperAngle = 0;
}

void moveArm(int base, int shoulder, int elbow, int wrist) {
  // Smooth movement to target positions
  moveServoSmooth(baseServo, baseAngle, base, 5);
  moveServoSmooth(shoulderServo, shoulderAngle, shoulder, 5);
  moveServoSmooth(elbowServo, elbowAngle, elbow, 5);
  moveServoSmooth(wristServo, wristAngle, wrist, 5);
  
  baseAngle = base;
  shoulderAngle = shoulder;
  elbowAngle = elbow;
  wristAngle = wrist;
}

void moveServoSmooth(Servo &servo, int currentAngle, int targetAngle, int step) {
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle += step) {
      servo.write(angle);
      delay(15);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle -= step) {
      servo.write(angle);
      delay(15);
    }
  }
  servo.write(targetAngle);
}

// ==================== PICKUP AND DROPOFF SEQUENCES ====================

void pickupSequence() {
  Serial.println("Starting pickup sequence...");
  
  // Stop the robot
  stopMotors();
  delay(500);

  // Move arm to pickup position (adjust angles based on your robot's design)
  moveArm(90, 45, 135, 90);  // Reach forward and down
  delay(500);

  // Open gripper
  openGripper();
  delay(300);

  // Lower arm to item
  moveArm(90, 60, 150, 90);
  delay(500);

  // Close gripper to grab item
  closeGripper();
  delay(500);

  // Lift arm with item
  moveArm(90, 30, 120, 90);
  delay(500);

  // Return to transport position
  moveArm(90, 45, 135, 90);
  delay(300);

  // Resume line following
  currentState = LINE_FOLLOWING;
  Serial.println("Pickup complete!");
}

void dropoffSequence() {
  Serial.println("Starting dropoff sequence...");
  
  // Stop the robot
  stopMotors();
  delay(500);

  // Move arm to dropoff position
  moveArm(90, 60, 150, 90);
  delay(500);

  // Open gripper to release item
  openGripper();
  delay(500);

  // Retract arm
  moveArm(90, 30, 120, 90);
  delay(500);

  // Return to default position
  initializeArm();
  delay(300);

  // Resume line following
  currentState = LINE_FOLLOWING;
  Serial.println("Dropoff complete!");
}

// ==================== UTILITY FUNCTIONS ====================

void setMotorSpeed(int speed) {
  motorSpeed = constrain(speed, 0, 255);
}

void setObstacleThreshold(int threshold) {
  obstacleThreshold = threshold;
}

// Function to manually trigger pickup (can be called from serial or button)
void triggerPickup() {
  currentState = PICKUP_ITEM;
}

// Function to manually trigger dropoff (can be called from serial or button)
void triggerDropoff() {
  currentState = DROPOFF_ITEM;
}

