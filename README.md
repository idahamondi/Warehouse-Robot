# Warehouse Robot - Autonomous Navigation System

An Arduino-based warehouse robot capable of autonomously navigating along predefined paths, avoiding obstacles, and transporting goods using a robotic arm.

## Features

- **Line Following**: Uses 5 IR line tracking sensors for precise path navigation
- **Obstacle Avoidance**: Ultrasonic sensor (HC-SR04) detects and avoids obstacles
- **Autonomous Navigation**: Integrated state machine for seamless operation
- **Robotic Arm Control**: 5-DOF robotic arm with gripper for picking and placing items
- **Motor Control**: Differential drive system with speed control via L298N motor driver

## Hardware Components

### Required Components

1. **Arduino Uno** (or compatible microcontroller)
2. **L298N Motor Driver Module** - For controlling DC motors
3. **DC Motors** (x2) - For robot movement
4. **Ultrasonic Sensor (HC-SR04)** - For obstacle detection
5. **IR Line Tracking Sensors** (x5) - For path following
6. **Servo Motors** (x5) - For robotic arm control
   - Base rotation servo
   - Shoulder joint servo
   - Elbow joint servo
   - Wrist joint servo
   - Gripper servo
7. **Power Supply** - 12V battery pack for motors, 5V for Arduino and servos
8. **Wheels and Chassis** - Robot platform

## Wiring Diagram

### Motor Driver (L298N) Connections

```
L298N Motor Driver → Arduino
─────────────────────────────────
ENA (Enable A)     → Pin 5 (PWM)
IN1                → Pin 7
IN2                → Pin 8
ENB (Enable B)     → Pin 6 (PWM)
IN3                → Pin 9
IN4                → Pin 10

L298N Motor Driver → Motors
─────────────────────────────────
OUT1, OUT2         → Left Motor
OUT3, OUT4         → Right Motor

L298N Power
─────────────────────────────────
VCC                → 5V (Arduino)
GND                → GND (Arduino)
12V                → 12V Battery (+)
GND                → 12V Battery (-)
```

### Ultrasonic Sensor (HC-SR04) Connections

```
HC-SR04 → Arduino
─────────────────────────────────
VCC     → 5V
GND     → GND
Trig    → Pin 12
Echo    → Pin 13
```

### IR Line Tracking Sensors

```
IR Sensors → Arduino
─────────────────────────────────
Left Sensor        → Pin 2
Left Center        → Pin 3
Center             → Pin 4
Right Center       → Pin 11
Right Sensor       → Pin 14 (A0)

All VCC            → 5V
All GND            → GND
```

**Note**: IR sensors typically have 3 pins:
- VCC (Power)
- GND (Ground)
- OUT (Signal) - Connect to Arduino digital pins

Some IR sensors may require pull-up resistors. Check your sensor datasheet.

### Servo Motor Connections

```
Servo Motors → Arduino
─────────────────────────────────
Base Servo         → Pin A1
Shoulder Servo     → Pin A2
Elbow Servo        → Pin A3
Wrist Servo        → Pin A4
Gripper Servo      → Pin A5

All VCC (Red)      → 5V (use external power supply for servos if needed)
All GND (Black)    → GND
All Signal (Yellow/White) → Respective Arduino pins
```

**Important**: Servo motors can draw significant current. Consider using an external 5V power supply for servos and connect the grounds together.

## Installation

1. **Install Arduino IDE**: Download and install Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)

2. **Install Servo Library**: 
   - The Servo library is included with Arduino IDE by default
   - If needed, install via: Sketch → Include Library → Servo

3. **Upload Code**:
   - Open `WarehouseRobot.ino` in Arduino IDE
   - Select your board: Tools → Board → Arduino Uno
   - Select your port: Tools → Port → (your Arduino port)
   - Click Upload button

## Configuration

### Adjusting Sensor Sensitivity

In the code, you can modify these parameters:

```cpp
int motorSpeed = 150;        // Base speed (0-255)
int turnSpeed = 100;         // Speed for turning
int obstacleThreshold = 15;  // Distance in cm to trigger obstacle avoidance
```

### IR Sensor Logic

The code assumes IR sensors return `LOW` when detecting a black line and `HIGH` when detecting a white surface. If your sensors work in reverse, modify the `readIRSensors()` function:

```cpp
void readIRSensors() {
  // Reverse logic if your sensors work differently
  irLeft = digitalRead(IR_SENSOR_LEFT);  // Remove the ! operator
  // ... etc
}
```

### Arm Positioning

Adjust the pickup and dropoff sequences in these functions:
- `pickupSequence()` - Modify angles for your arm's reach
- `dropoffSequence()` - Modify angles for dropoff position
- `initializeArm()` - Set default arm position

## Usage

### Basic Operation

1. **Power On**: Connect the robot to power
2. **Place on Line**: Position the robot on a black line track
3. **Automatic Operation**: The robot will:
   - Follow the line automatically
   - Detect and avoid obstacles
   - Navigate along the path

### Manual Control (Optional)

You can add serial commands for manual control. Add this to the `loop()` function:

```cpp
if (Serial.available() > 0) {
  char command = Serial.read();
  switch (command) {
    case 'p': triggerPickup(); break;
    case 'd': triggerDropoff(); break;
    case 's': currentState = IDLE; break;
    case 'f': currentState = LINE_FOLLOWING; break;
  }
}
```

### Robot States

The robot operates in different states:

- **LINE_FOLLOWING**: Default state, follows the line path
- **OBSTACLE_AVOIDANCE**: Activated when obstacle is detected
- **PICKUP_ITEM**: Executes pickup sequence
- **DROPOFF_ITEM**: Executes dropoff sequence
- **IDLE**: Robot stops all movement

## Troubleshooting

### Robot Doesn't Follow Line

1. **Check IR Sensors**: Ensure all sensors are properly connected and powered
2. **Calibrate Sensors**: Adjust sensor height above the line (typically 2-5mm)
3. **Invert Logic**: Some IR sensors work in reverse - modify `readIRSensors()` function
4. **Line Contrast**: Ensure sufficient contrast between line and surface (black line on white surface works best)

### Motors Not Working

1. **Check Power**: Ensure 12V battery is connected to L298N
2. **Check Connections**: Verify all motor driver connections
3. **Test Motors**: Manually test each motor by setting pins HIGH/LOW
4. **Enable Pins**: Ensure ENA and ENB pins are receiving PWM signals

### Ultrasonic Sensor Issues

1. **Check Wiring**: Verify Trig and Echo pins are correct
2. **Power Supply**: Ensure sensor has stable 5V power
3. **Obstacle Distance**: Adjust `obstacleThreshold` if sensor is too sensitive
4. **Sensor Range**: HC-SR04 works best between 2cm and 400cm

### Servo Motors Not Moving

1. **Power Supply**: Servos may need external 5V power supply
2. **Check Connections**: Verify signal, power, and ground connections
3. **Current Limit**: Too many servos may exceed Arduino's current limit - use external power
4. **PWM Pins**: Ensure servos are connected to PWM-capable pins

### Robot Vibrates or Shakes

1. **Reduce Speed**: Lower `motorSpeed` value
2. **Smooth Turns**: Adjust line following algorithm sensitivity
3. **Mechanical Issues**: Check for loose connections or mechanical problems

## Code Structure

- **Motor Control**: Functions for forward, backward, turn, and stop
- **Sensor Reading**: Functions to read ultrasonic and IR sensors
- **Line Following**: Algorithm for smooth path following
- **Obstacle Avoidance**: Logic to detect and navigate around obstacles
- **Arm Control**: Functions for robotic arm movement and gripper control
- **State Machine**: Main control loop managing robot states

## Customization

### Adding More Sensors

To add more IR sensors, simply:
1. Define new pin in the pin definitions section
2. Add reading in `readIRSensors()` function
3. Update `followLine()` algorithm to use new sensor

### Modifying Arm Movements

Edit the `pickupSequence()` and `dropoffSequence()` functions to customize arm movements for your specific robot design and workspace requirements.

### Speed Calibration

Adjust motor speeds based on your:
- Motor specifications
- Battery voltage
- Surface conditions
- Desired navigation speed

## Safety Notes

1. **Power Supply**: Use appropriate power supply ratings to avoid damage
2. **Current Protection**: Consider adding fuses for motor and servo power lines
3. **Testing**: Test in a safe area before full operation
4. **Emergency Stop**: Consider adding an emergency stop button
5. **Battery**: Use proper battery management to prevent over-discharge

## Future Enhancements

- Wireless communication for remote control
- Multiple pickup/dropoff locations
- RFID tag reading for item identification
- Camera integration for visual navigation
- Battery level monitoring
- LED indicators for status display
- Sound alerts for operations

## License

This project is open source and available for educational and personal use.

## Contributors

Developed as a group project for warehouse automation and logistics.

## Support

For issues or questions, please refer to:
- Arduino documentation: https://www.arduino.cc/reference/
- L298N datasheet
- HC-SR04 ultrasonic sensor documentation
- Servo motor specifications

---

**Note**: This code is designed for Arduino Uno. For other boards (Nano, Mega, etc.), adjust pin assignments as needed. Some boards have different PWM pin capabilities.

