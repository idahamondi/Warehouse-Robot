# Quick Wiring Reference Guide

## Pin Assignment Summary

### Digital Pins Used
- **Pin 2**: IR Sensor Left
- **Pin 3**: IR Sensor Left Center
- **Pin 4**: IR Sensor Center
- **Pin 5**: Motor Left ENA (PWM)
- **Pin 6**: Motor Right ENB (PWM)
- **Pin 7**: Motor Left IN1
- **Pin 8**: Motor Left IN2
- **Pin 9**: Motor Right IN3
- **Pin 10**: Motor Right IN4
- **Pin 11**: IR Sensor Right Center
- **Pin 12**: Ultrasonic Sensor Trig
- **Pin 13**: Ultrasonic Sensor Echo
- **Pin 14 (A0)**: IR Sensor Right

### Analog Pins Used (as Digital)
- **Pin A1**: Servo Base
- **Pin A2**: Servo Shoulder
- **Pin A3**: Servo Elbow
- **Pin A4**: Servo Wrist
- **Pin A5**: Servo Gripper

## Connection Checklist

### Power Connections
- [ ] 12V Battery (+) to L298N 12V input
- [ ] 12V Battery (-) to L298N GND and Arduino GND (common ground)
- [ ] L298N 5V to Arduino 5V (optional, if using L298N's 5V regulator)
- [ ] External 5V power supply for servos (recommended)
- [ ] All GND connections tied together (common ground)

### Motor Connections
- [ ] Left Motor wires to L298N OUT1 and OUT2
- [ ] Right Motor wires to L298N OUT3 and OUT4
- [ ] L298N ENA to Arduino Pin 5
- [ ] L298N IN1 to Arduino Pin 7
- [ ] L298N IN2 to Arduino Pin 8
- [ ] L298N ENB to Arduino Pin 6
- [ ] L298N IN3 to Arduino Pin 9
- [ ] L298N IN4 to Arduino Pin 10

### Sensor Connections
- [ ] HC-SR04 VCC to 5V
- [ ] HC-SR04 GND to GND
- [ ] HC-SR04 Trig to Arduino Pin 12
- [ ] HC-SR04 Echo to Arduino Pin 13
- [ ] All 5 IR Sensors VCC to 5V
- [ ] All 5 IR Sensors GND to GND
- [ ] IR Sensor outputs to respective Arduino pins (2, 3, 4, 11, 14)

### Servo Connections
- [ ] Base Servo signal to Pin A1
- [ ] Shoulder Servo signal to Pin A2
- [ ] Elbow Servo signal to Pin A3
- [ ] Wrist Servo signal to Pin A4
- [ ] Gripper Servo signal to Pin A5
- [ ] All servos VCC to 5V (external power recommended)
- [ ] All servos GND to common GND

## Testing Each Component

### Test Motors
1. Upload code with motor test function
2. Verify each motor spins in correct direction
3. Check PWM speed control works

### Test Ultrasonic Sensor
1. Open Serial Monitor (9600 baud)
2. Check distance readings
3. Verify obstacle detection triggers

### Test IR Sensors
1. Place robot over black line
2. Check Serial output for sensor states
3. Verify sensors detect line correctly

### Test Servos
1. Upload code with servo test
2. Verify each servo moves to position
3. Check gripper opens and closes

## Common Wiring Mistakes

1. **Wrong Motor Polarity**: If motors spin backward, swap motor wires
2. **Missing Common Ground**: All components must share common GND
3. **Insufficient Power**: Servos may need external 5V supply
4. **Wrong Sensor Logic**: Some IR sensors work in reverse - adjust code
5. **Loose Connections**: Ensure all connections are secure

## Power Requirements

- **Arduino**: 5V via USB or Vin (7-12V recommended)
- **L298N Motors**: 12V, 2A minimum (depends on motors)
- **Servos**: 5V, 1-2A total (use external supply for multiple servos)
- **Sensors**: 5V, minimal current draw

## Safety Notes

- Always connect GND first when wiring
- Use appropriate wire gauge for motors (18-20 AWG)
- Add fuses for motor power lines if possible
- Test each component individually before full integration
- Double-check all connections before powering on

