/*
 * Component Test Sketch for Warehouse Robot
 * 
 * Use this sketch to test individual components before running the full robot code.
 * Comment/uncomment the test sections as needed.
 */

// ==================== PIN DEFINITIONS ====================
// Motor Driver Pins
#define MOTOR_LEFT_ENA 5
#define MOTOR_LEFT_IN1 7
#define MOTOR_LEFT_IN2 8
#define MOTOR_RIGHT_ENB 6
#define MOTOR_RIGHT_IN3 9
#define MOTOR_RIGHT_IN4 10

// Ultrasonic Sensor Pins
#define TRIGGER_PIN 12
#define ECHO_PIN 13

// IR Sensor Pins
#define IR_SENSOR_LEFT 2
#define IR_SENSOR_LEFT_CENTER 3
#define IR_SENSOR_CENTER 4
#define IR_SENSOR_RIGHT_CENTER 11
#define IR_SENSOR_RIGHT 14

// Servo Pins
#define SERVO_BASE A1
#define SERVO_SHOULDER A2
#define SERVO_ELBOW A3
#define SERVO_WRIST A4
#define SERVO_GRIPPER A5

#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

void setup() {
  Serial.begin(9600);
  Serial.println("=== Warehouse Robot Component Test ===");
  Serial.println("Select test to run:");
  Serial.println("1 - Motor Test");
  Serial.println("2 - Ultrasonic Sensor Test");
  Serial.println("3 - IR Sensor Test");
  Serial.println("4 - Servo Test");
  Serial.println("5 - All Tests");
  
  // Initialize all components
  setupMotors();
  setupUltrasonic();
  setupIRSensors();
  setupServos();
  
  delay(2000);
}

void loop() {
  // Uncomment the test you want to run:
  
  // testMotors();
  // testUltrasonicSensor();
  // testIRSensors();
  // testServos();
  testAll();
  
  delay(1000);
}

// ==================== SETUP FUNCTIONS ====================

void setupMotors() {
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  stopMotors();
}

void setupUltrasonic() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void setupIRSensors() {
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_LEFT_CENTER, INPUT);
  pinMode(IR_SENSOR_CENTER, INPUT);
  pinMode(IR_SENSOR_RIGHT_CENTER, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
}

void setupServos() {
  baseServo.attach(SERVO_BASE);
  shoulderServo.attach(SERVO_SHOULDER);
  elbowServo.attach(SERVO_ELBOW);
  wristServo.attach(SERVO_WRIST);
  gripperServo.attach(SERVO_GRIPPER);
}

// ==================== MOTOR TEST ====================

void testMotors() {
  Serial.println("\n=== Motor Test ===");
  
  Serial.println("Testing Left Motor Forward...");
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, 150);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Left Motor Backward...");
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_ENA, 150);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Right Motor Forward...");
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, 150);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Right Motor Backward...");
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_RIGHT_ENB, 150);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Both Motors Forward...");
  moveForward(150);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Turn Left...");
  turnLeft(100);
  delay(2000);
  stopMotors();
  delay(500);
  
  Serial.println("Testing Turn Right...");
  turnRight(100);
  delay(2000);
  stopMotors();
  
  Serial.println("Motor test complete!\n");
}

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

// ==================== ULTRASONIC SENSOR TEST ====================

void testUltrasonicSensor() {
  Serial.println("\n=== Ultrasonic Sensor Test ===");
  
  for (int i = 0; i < 10; i++) {
    int distance = readUltrasonicSensor();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    if (distance < 15) {
      Serial.println("OBSTACLE DETECTED!");
    }
    
    delay(500);
  }
  
  Serial.println("Ultrasonic test complete!\n");
}

int readUltrasonicSensor() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  
  return distance;
}

// ==================== IR SENSOR TEST ====================

void testIRSensors() {
  Serial.println("\n=== IR Sensor Test ===");
  Serial.println("Place robot over line and observe sensor readings:");
  Serial.println("L | LC | C | RC | R");
  Serial.println("---------------------");
  
  for (int i = 0; i < 50; i++) {
    bool irLeft = !digitalRead(IR_SENSOR_LEFT);
    bool irLeftCenter = !digitalRead(IR_SENSOR_LEFT_CENTER);
    bool irCenter = !digitalRead(IR_SENSOR_CENTER);
    bool irRightCenter = !digitalRead(IR_SENSOR_RIGHT_CENTER);
    bool irRight = !digitalRead(IR_SENSOR_RIGHT);
    
    Serial.print(irLeft ? "1" : "0");
    Serial.print(" | ");
    Serial.print(irLeftCenter ? "1" : "0");
    Serial.print("  | ");
    Serial.print(irCenter ? "1" : "0");
    Serial.print(" | ");
    Serial.print(irRightCenter ? "1" : "0");
    Serial.print("  | ");
    Serial.println(irRight ? "1" : "0");
    
    delay(200);
  }
  
  Serial.println("IR sensor test complete!\n");
}

// ==================== SERVO TEST ====================

void testServos() {
  Serial.println("\n=== Servo Test ===");
  
  Serial.println("Testing Base Servo...");
  for (int angle = 0; angle <= 180; angle += 10) {
    baseServo.write(angle);
    Serial.print("Base angle: ");
    Serial.println(angle);
    delay(100);
  }
  baseServo.write(90);
  delay(500);
  
  Serial.println("Testing Shoulder Servo...");
  for (int angle = 0; angle <= 180; angle += 10) {
    shoulderServo.write(angle);
    Serial.print("Shoulder angle: ");
    Serial.println(angle);
    delay(100);
  }
  shoulderServo.write(90);
  delay(500);
  
  Serial.println("Testing Elbow Servo...");
  for (int angle = 0; angle <= 180; angle += 10) {
    elbowServo.write(angle);
    Serial.print("Elbow angle: ");
    Serial.println(angle);
    delay(100);
  }
  elbowServo.write(90);
  delay(500);
  
  Serial.println("Testing Wrist Servo...");
  for (int angle = 0; angle <= 180; angle += 10) {
    wristServo.write(angle);
    Serial.print("Wrist angle: ");
    Serial.println(angle);
    delay(100);
  }
  wristServo.write(90);
  delay(500);
  
  Serial.println("Testing Gripper Servo...");
  Serial.println("Closing gripper...");
  for (int angle = 180; angle >= 0; angle -= 10) {
    gripperServo.write(angle);
    delay(100);
  }
  delay(500);
  
  Serial.println("Opening gripper...");
  for (int angle = 0; angle <= 180; angle += 10) {
    gripperServo.write(angle);
    delay(100);
  }
  gripperServo.write(0);
  delay(500);
  
  Serial.println("Servo test complete!\n");
}

// ==================== ALL TESTS ====================

void testAll() {
  static unsigned long lastTest = 0;
  static int testNum = 0;
  
  if (millis() - lastTest > 15000) {  // Run each test for 15 seconds
    lastTest = millis();
    testNum++;
    
    switch (testNum) {
      case 1:
        Serial.println("\n>>> Starting Motor Test <<<");
        testMotors();
        break;
      case 2:
        Serial.println("\n>>> Starting Ultrasonic Test <<<");
        testUltrasonicSensor();
        break;
      case 3:
        Serial.println("\n>>> Starting IR Sensor Test <<<");
        testIRSensors();
        break;
      case 4:
        Serial.println("\n>>> Starting Servo Test <<<");
        testServos();
        testNum = 0;  // Reset to loop tests
        break;
    }
  }
  
  // Continuous monitoring
  if (testNum == 2) {
    int distance = readUltrasonicSensor();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(500);
  }
  
  if (testNum == 3) {
    bool irLeft = !digitalRead(IR_SENSOR_LEFT);
    bool irLeftCenter = !digitalRead(IR_SENSOR_LEFT_CENTER);
    bool irCenter = !digitalRead(IR_SENSOR_CENTER);
    bool irRightCenter = !digitalRead(IR_SENSOR_RIGHT_CENTER);
    bool irRight = !digitalRead(IR_SENSOR_RIGHT);
    
    Serial.print("IR: ");
    Serial.print(irLeft ? "1" : "0");
    Serial.print(irLeftCenter ? "1" : "0");
    Serial.print(irCenter ? "1" : "0");
    Serial.print(irRightCenter ? "1" : "0");
    Serial.println(irRight ? "1" : "0");
    delay(200);
  }
}

