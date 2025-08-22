
#include <PID_v1.h>

#define PULSES_PER_MOTOR_REV 13
#define GEAR_RATIO 480
#define PULSES_PER_GEARBOX_REV (PULSES_PER_MOTOR_REV * GEAR_RATIO)  // 6240 pulses/rev
#define DEGREES_PER_PULSE (360.0 / PULSES_PER_GEARBOX_REV)          // ~0.0577 degrees/pulse

#define ENCODER_PPR 500
#define SCREW_PITCH 10.0
#define GEAR_RATIO_M3 1.0
const float PULSES_PER_MM = (ENCODER_PPR * GEAR_RATIO_M3) / SCREW_PITCH;
const float MM_PER_PULSE = SCREW_PITCH / (ENCODER_PPR * GEAR_RATIO_M3);

const float X1 = -200.0;
const float Y1 = 0.0;
const float R2 = 260.0;
const float R3 = 390.0;

// Motor pin definitions
// Motor 1 (Left motor) - L298N Channel A
#define MOTOR1_ENA_PIN 7
#define MOTOR1_IN1_PIN 26
#define MOTOR1_IN2_PIN 27
#define ENCODER1_A_PIN 21
#define ENCODER1_B_PIN 20
#define SENSOR1_PIN 8

// Motor 2 (Right motor) - L298N Channel B
#define MOTOR2_ENA_PIN 9
#define MOTOR2_IN1_PIN 28
#define MOTOR2_IN2_PIN 29
#define ENCODER2_A_PIN 18
#define ENCODER2_B_PIN 19
#define SENSOR2_PIN 13

// Motor 3 (Z-axis motor) - keeping original
#define MOTOR3_IN1 30
#define MOTOR3_IN2 31
#define MOTOR3_ENA 5
#define ENCODER3_A 3
#define ENCODER3_B 2

// Improved PID Controller Structure
struct PIDController {
  float Kp;
  float Ki;
  float Kd;
  
  float previousError = 0;
  float integral = 0;
  unsigned long lastTime = 0;
  
  PIDController(float kp = 11.0, float ki = 2.0, float kd = 3.0) : Kp(kp), Ki(ki), Kd(kd) {}
};

struct MotorController {
  int ena_pin, in1_pin, in2_pin;
  int encA_pin, encB_pin;
  int sensor_pin;
  
  volatile long encoderPosition = 0;
  long targetPosition = 0;
  long homePosition = 0;
  
  int motorSpeed = 0;
  int maxSpeed = 255;
  bool motorDirection = true;
  bool isMoving = false;
  bool homed = false;
  
  PIDController pid;
  
  String name;
  
  int deadZone = 5;  // pulses
}; 

// Motor instances
MotorController motor1;
MotorController motor2;

// Motor 3 - keeping original structure for Z-axis
struct Motor {
  int in1, in2, ena;
  int encoderA, encoderB;
  volatile long encoderCount;
  float currentPosition;
  float targetPosition;
  bool homed;
};

Motor motor3 = {MOTOR3_IN1, MOTOR3_IN2, MOTOR3_ENA, ENCODER3_A, ENCODER3_B, 0, 0, 0, false};

// Original PID for Motor 3
double pid3_input, pid3_output, pid3_setpoint;
PID pid3(&pid3_input, &pid3_output, &pid3_setpoint, 1.0, 0.1, 0.5, DIRECT);

// System state
bool systemHomed = false;
bool movementActive = false;
bool positionMonitoring = false;
unsigned long lastPrintTime = 0;
unsigned long lastStatusUpdate = 0;
const unsigned long PRINT_INTERVAL = 250;

void setup() {
  Serial.begin(115200);
  Serial.println("Enhanced 3-Motor Control System Starting...");
  
  // Initialize motor 1 parameters
  motor1.ena_pin = MOTOR1_ENA_PIN;
  motor1.in1_pin = MOTOR1_IN1_PIN;
  motor1.in2_pin = MOTOR1_IN2_PIN;
  motor1.encA_pin = ENCODER1_A_PIN;
  motor1.encB_pin = ENCODER1_B_PIN;
  motor1.sensor_pin = SENSOR1_PIN;
  motor1.name = "Motor1";
  motor1.pid = PIDController(11.0, 2.0, 3.0);  // Tuned PID values
  motor1.deadZone = 5;
  
  // Initialize motor 2 parameters
  motor2.ena_pin = MOTOR2_ENA_PIN;
  motor2.in1_pin = MOTOR2_IN1_PIN;
  motor2.in2_pin = MOTOR2_IN2_PIN;
  motor2.encA_pin = ENCODER2_A_PIN;
  motor2.encB_pin = ENCODER2_B_PIN;
  motor2.sensor_pin = SENSOR2_PIN;
  motor2.name = "Motor2";
  motor2.pid = PIDController(11.0, 2.0, 3.0);  // Tuned PID values
  motor2.deadZone = 5;
  
  initializePins();
  initializeInterrupts();
  initializePID();
  
  Serial.println("System initialized. Type 'help' for commands.");
}

void loop() {
  updateMotorPositions();
  handleSerialCommands();
  
  // Run improved position control for motors 1 & 2
  runPositionControl();
  
  // Original control for motor 3
  if (movementActive) {
    updateMotor3Control();
    
    // Print positions during movement
    if (positionMonitoring && (millis() - lastPrintTime >= PRINT_INTERVAL)) {
      printCurrentPositions();
      lastPrintTime = millis();
    }
  }
  
  // Print status periodically
  if (millis() - lastStatusUpdate > 500) {
    printQuickStatus();
    lastStatusUpdate = millis();
  }
  
  delay(10);
}

void initializePins() {
  // Initialize Motor 1
  pinMode(motor1.ena_pin, OUTPUT);
  pinMode(motor1.in1_pin, OUTPUT);
  pinMode(motor1.in2_pin, OUTPUT);
  pinMode(motor1.encA_pin, INPUT_PULLUP);
  pinMode(motor1.encB_pin, INPUT_PULLUP);
  pinMode(motor1.sensor_pin, INPUT_PULLUP);
  
  // Initialize Motor 2
  pinMode(motor2.ena_pin, OUTPUT);
  pinMode(motor2.in1_pin, OUTPUT);
  pinMode(motor2.in2_pin, OUTPUT);
  pinMode(motor2.encA_pin, INPUT_PULLUP);
  pinMode(motor2.encB_pin, INPUT_PULLUP);
  pinMode(motor2.sensor_pin, INPUT_PULLUP);
  
  // Initialize Motor 3 (original)
  pinMode(motor3.in1, OUTPUT);
  pinMode(motor3.in2, OUTPUT);
  pinMode(motor3.ena, OUTPUT);
  pinMode(motor3.encoderA, INPUT_PULLUP);
  pinMode(motor3.encoderB, INPUT_PULLUP);
  
  // Stop all motors initially
  stopMotor(motor1);
  stopMotor(motor2);
  stopMotorOriginal(motor3);
}

void initializeInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder3ISR, CHANGE);
}

void initializePID() {
  // Original PID for motor 3
  pid3.SetMode(AUTOMATIC);
  pid3.SetOutputLimits(-255, 255);
}

// Enhanced encoder interrupt service routines
void encoder1ISR() {
  updateEncoder(motor1);
}

void encoder2ISR() {
  updateEncoder(motor2);
}

void encoder3ISR() {
  if (digitalRead(ENCODER3_A) == digitalRead(ENCODER3_B)) {
    motor3.encoderCount++;
  } else {
    motor3.encoderCount--;
  }
}

void updateEncoder(MotorController& motor) {
  bool channelA = digitalRead(motor.encA_pin);
  bool channelB = digitalRead(motor.encB_pin);
  
  // Use static variables to track last state
  static bool lastChannelA1 = false;
  static bool lastChannelA2 = false;
  
  bool* lastChannelA;
  if (&motor == &motor1) lastChannelA = &lastChannelA1;
  else lastChannelA = &lastChannelA2;
  
  if (channelA != *lastChannelA) {
    if (channelA == channelB) {
      motor.encoderPosition--;  // Counter-clockwise
    } else {
      motor.encoderPosition++;  // Clockwise
    }
  }
  
  *lastChannelA = channelA;
}

// Enhanced position control for motors 1 & 2
void runPositionControl() {
  controlMotor(motor1);
  controlMotor(motor2);
}

void controlMotor(MotorController& motor) {
  if (!motor.isMoving) return;
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - motor.pid.lastTime) / 1000.0;
  
  if (deltaTime < 0.01) return; // Minimum control interval
  
  // Calculate error in pulses
  long error = motor.targetPosition - motor.encoderPosition;
  
  // Check if we've reached the target (within dead zone)
  if (abs(error) < motor.deadZone) {
    stopMotor(motor);
    motor.isMoving = false;
    Serial.print(motor.name);
    Serial.println(" reached target position");
    return;
  }
  
  // PID calculations
  motor.pid.integral += error * deltaTime;
  float derivative = (error - motor.pid.previousError) / deltaTime;
  
  // Anti-windup for integral term
  motor.pid.integral = constrain(motor.pid.integral, -1000, 1000);
  
  // Calculate PID output
  float output = motor.pid.Kp * error + motor.pid.Ki * motor.pid.integral + motor.pid.Kd * derivative;
  
  // Convert to motor control values
  motor.motorSpeed = constrain(abs(output), 30, motor.maxSpeed);  // Minimum speed of 30
  motor.motorDirection = (output >= 0);
  
  // Apply motor control
  setMotorSpeed(motor, motor.motorSpeed, motor.motorDirection);
  
  // Update for next iteration
  motor.pid.previousError = error;
  motor.pid.lastTime = currentTime;
}

void setMotorSpeed(MotorController& motor, int speed, bool direction) {
  if (direction) {
    digitalWrite(motor.in1_pin, HIGH);
    digitalWrite(motor.in2_pin, LOW);
  } else {
    digitalWrite(motor.in1_pin, LOW);
    digitalWrite(motor.in2_pin, HIGH);
  }
  analogWrite(motor.ena_pin, speed);
}

void stopMotor(MotorController& motor) {
  digitalWrite(motor.in1_pin, LOW);
  digitalWrite(motor.in2_pin, LOW);
  analogWrite(motor.ena_pin, 0);
  
  motor.motorSpeed = 0;
  motor.isMoving = false;
  motor.targetPosition = motor.encoderPosition;  // Update target to current position
  motor.pid.integral = 0;  // Reset integral term
}

// Original motor 3 functions
void updateMotorPositions() {
  motor3.currentPosition = motor3.encoderCount * MM_PER_PULSE;
}

void updateMotor3Control() {
  if (abs(motor3.targetPosition - motor3.currentPosition) > 2.0) {
    pid3_input = motor3.currentPosition;
    pid3_setpoint = motor3.targetPosition;
    pid3.Compute();
    moveMotorOriginal(motor3, (int)pid3_output);
  } else {
    stopMotorOriginal(motor3);
    movementActive = false;
    positionMonitoring = false;
    Serial.println("Motor 3 reached target position");
  }
}

void moveMotorOriginal(Motor& motor, int speed) {
  if (speed > 0) {
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
    analogWrite(motor.ena, min(abs(speed), 255));
  } else if (speed < 0) {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
    analogWrite(motor.ena, min(abs(speed), 255));
  } else {
    stopMotorOriginal(motor);
  }
}

void stopMotorOriginal(Motor& motor) {
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
  analogWrite(motor.ena, 0);
}

// Enhanced movement functions
void moveMotorToPosition(MotorController& motor, float degrees) {
  long targetPulses = degreesToPulses(degrees);
  motor.targetPosition = targetPulses;
  motor.isMoving = true;
  
  // Reset PID parameters
  motor.pid.integral = 0;
  motor.pid.previousError = 0;
  motor.pid.lastTime = millis();
  
  Serial.print("Moving ");
  Serial.print(motor.name);
  Serial.print(" to ");
  Serial.print(degrees, 2);
  Serial.print("° (");
  Serial.print(targetPulses);
  Serial.println(" pulses)");
}

void moveMotor3ToDistance(float targetDistance) {
  if (!systemHomed) {
    Serial.println("System must be homed first!");
    return;
  }
  
  Serial.print("Moving Motor 3 to distance: ");
  Serial.print(targetDistance);
  Serial.println(" mm");
  
  motor3.targetPosition = targetDistance;
  movementActive = true;
  positionMonitoring = true;
  lastPrintTime = millis();
}

// Enhanced inverse kinematics
void calculateInverseKinematics(float x, float y) {
  float dx1 = x - X1;
  float dy1 = y - Y1;
  float d1 = sqrt(dx1 * dx1 + dy1 * dy1);
  float d2 = sqrt(x * x + y * y);
  
  if (d1 > R2 + R3 || d1 < abs(R2 - R3) || d2 > R2 + R3 || d2 < abs(R2 - R3)) {
    Serial.println("ERROR: XY position unreachable!");
    Serial.print("Distance 1: "); Serial.print(d1);
    Serial.print(" (valid range: "); Serial.print(abs(R2 - R3));
    Serial.print(" - "); Serial.print(R2 + R3); Serial.println(")");
    Serial.print("Distance 2: "); Serial.print(d2);
    Serial.print(" (valid range: "); Serial.print(abs(R2 - R3));
    Serial.print(" - "); Serial.print(R2 + R3); Serial.println(")");
    return;
  }
  
  float alpha2 = atan2(dy1, dx1);
  float beta2 = acos((R2 * R2 + d1 * d1 - R3 * R3) / (2 * R2 * d1));
  float theta2 = PI - alpha2 - beta2;
  float theta2Deg = theta2 * 180.0 / PI;
  
  float beta4 = atan2(y, -x);
  float alpha4 = acos((R2 * R2 + d2 * d2 - R3 * R3) / (2 * R2 * d2));
  float theta4 = PI - alpha4 - beta4;
  float theta4Deg = theta4 * 180.0 / PI;
  
  Serial.print("Calculated XY angles: ");
  Serial.print(theta2Deg, 2);
  Serial.print("°, ");
  Serial.print(theta4Deg, 2);
  Serial.println("°");
  
  if (!systemHomed) {
    Serial.println("System must be homed first!");
    return;
  }
  
  moveMotorToPosition(motor1, theta2Deg);
  moveMotorToPosition(motor2, theta4Deg);
}

// Enhanced serial command handling
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "help") {
      printHelp();
    }
    else if (command == "home") {
      homeSystem();
    }
    else if (command == "status") {
      printStatus();
    }
    else if (command == "stop") {
      stopAllMotors();
    }
    else if (command == "pos") {
      printCurrentPositions();
    }
    else if (command == "debug") {
      printMovementDebug();
    }
    else if (command == "monitor on") {
      positionMonitoring = true;
      Serial.println("Position monitoring enabled");
    }
    else if (command == "monitor off") {
      positionMonitoring = false;
      Serial.println("Position monitoring disabled");
    }
    else if (command.startsWith("xy ")) {
      parseXYCommand(command);
    }
    else if (command.startsWith("z ")) {
      parseZCommand(command);
    }
    else if (command.startsWith("pid ")) {
      parsePIDCommand(command);
    }
    else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}

// Utility functions
long degreesToPulses(float degrees) {
  return (long)(degrees / DEGREES_PER_PULSE);
}

float pulsesToDegrees(long pulses) {
  return pulses * DEGREES_PER_PULSE;
}

// Additional command parsers
void parseXYCommand(String command) {
  int spaceIndex1 = command.indexOf(' ');
  int spaceIndex2 = command.indexOf(' ', spaceIndex1 + 1);
  
  if (spaceIndex1 != -1 && spaceIndex2 != -1) {
    float x = command.substring(spaceIndex1 + 1, spaceIndex2).toFloat();
    float y = command.substring(spaceIndex2 + 1).toFloat();
    
    Serial.print("Moving to XY position: (");
    Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");
    
    calculateInverseKinematics(x, y);
  } else {
    Serial.println("Invalid XY command format. Use: xy <x> <y>");
  }
}

void parseZCommand(String command) {
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex != -1) {
    float z = command.substring(spaceIndex + 1).toFloat();
    moveMotor3ToDistance(z);
  } else {
    Serial.println("Invalid Z command format. Use: z <distance>");
  }
}

void parsePIDCommand(String command) {
  int firstSpace = command.indexOf(' ', 4);
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  
  if (firstSpace > 0 && secondSpace > 0) {
    float kp = command.substring(4, firstSpace).toFloat();
    float ki = command.substring(firstSpace + 1, secondSpace).toFloat();
    float kd = command.substring(secondSpace + 1).toFloat();
    
    motor1.pid.Kp = kp;
    motor1.pid.Ki = ki;
    motor1.pid.Kd = kd;
    motor2.pid.Kp = kp;
    motor2.pid.Ki = ki;
    motor2.pid.Kd = kd;
    
    Serial.print("PID parameters set for M1&M2 - Kp:");
    Serial.print(kp); Serial.print(" Ki:");
    Serial.print(ki); Serial.print(" Kd:");
    Serial.println(kd);
  } else {
    Serial.println("Usage: pid <kp> <ki> <kd>");
  }
}

void homeSystem() {
  Serial.println("Starting enhanced homing sequence...");
  Serial.println("Homing Motors 1 and 2 simultaneously...");
  
  motor1.homed = false;
  motor2.homed = false;
  
  while (!motor1.homed || !motor2.homed) {
    if (!motor1.homed) {
      if (digitalRead(motor1.sensor_pin) == HIGH) {
        setMotorSpeed(motor1, 100, false);  // Move backward
      } else {
        stopMotor(motor1);
        motor1.encoderPosition = 0;
        motor1.targetPosition = 0;
        motor1.homed = true;
        Serial.println("Motor 1 homed successfully");
      }
    }
    
    if (!motor2.homed) {
      if (digitalRead(motor2.sensor_pin) == HIGH) {
        setMotorSpeed(motor2, 100, false);  // Move backward
      } else {
        stopMotor(motor2);
        motor2.encoderPosition = 0;
        motor2.targetPosition = 0;
        motor2.homed = true;
        Serial.println("Motor 2 homed successfully");
      }
    }
    
    delay(10);
  }
  
  motor3.encoderCount = 0;
  motor3.currentPosition = 0;
  motor3.homed = true;
  
  systemHomed = true;
  Serial.println("Enhanced homing complete!");
}

void stopAllMotors() {
  stopMotor(motor1);
  stopMotor(motor2);
  stopMotorOriginal(motor3);
  movementActive = false;
  positionMonitoring = false;
  Serial.println("All motors stopped");
}
