/*
 * ============================================================================
 * COMPLETE DEBUG + NORMAL COMMANDS VERSION
 * ============================================================================
 * 
 * TÍNH NĂNG:
 * - Tất cả lệnh debug chuyên sâu
 * - Tất cả lệnh điều khiển bình thường
 * - Test hardware + điều khiển robot
 * - Force test + smooth trajectory
 * ============================================================================
 */

 #include <Arduino.h>
 #include <math.h>
 
 // ========== HARDWARE PINS ==========
 #define ENC1_A          20
 #define ENC1_B          21
 #define MOT1_PWM        7
 #define MOT1_DIR1       26
 #define MOT1_DIR2       27
 #define SENSOR1         8
 
 #define ENC2_A          18
 #define ENC2_B          19
 #define MOT2_PWM        9
 #define MOT2_DIR1       28
 #define MOT2_DIR2       29
 #define SENSOR2         13
 
 #define ESTOP_PIN       12
 
 // ========== PARAMETERS ==========
 #define ENCODER_CPR     500
 #define MAX_SPEED       200
 #define MIN_SPEED       60
 #define TEST_SPEED      120
 #define HOMING_SPEED    100
 #define ACCEL_RATE      8
 #define POSITION_TOL    1.5
 
 const float COUNTS_PER_DEGREE = ENCODER_CPR / 360.0;
 
 // ========== TRAJECTORY STRUCTURE ==========
 struct TrajectoryPoint {
   float angle1;
   float angle2;
   float speed_percent;
   unsigned long duration_ms;
 };
 
 // ========== SMOOTH CONTROLLER ==========
 struct SmoothController {
   float current_position;
   float target_position;
   float current_velocity;
   bool is_moving;
   bool reached_target;
   
   void reset() {
     current_velocity = 0;
     is_moving = false;
     reached_target = false;
   }
   
   void setTarget(float target) {
     target_position = target;
     is_moving = true;
     reached_target = false;
   }
   
   float updateSmooth(float actual_pos) {
     current_position = actual_pos;
     
     if (!is_moving) return 0;
     
     float error = target_position - current_position;
     
     if (abs(error) < POSITION_TOL) {
       reset();
       reached_target = true;
       return 0;
     }
     
     float desired_velocity = min(150.0f, abs(error) * 12);
     desired_velocity = max(desired_velocity, (float)MIN_SPEED);
     
     if (current_velocity < desired_velocity) {
       current_velocity += ACCEL_RATE;
       if (current_velocity > desired_velocity) {
         current_velocity = desired_velocity;
       }
     } else if (current_velocity > desired_velocity) {
       current_velocity -= ACCEL_RATE;
       if (current_velocity < MIN_SPEED) {
         current_velocity = MIN_SPEED;
       }
     }
     
     return (error > 0) ? current_velocity : -current_velocity;
   }
   
   bool isMoving() { return is_moving; }
 };
 
 // ========== GLOBAL VARIABLES ==========
 volatile long encoder1_count = 0;
 volatile long encoder2_count = 0;
 
 float motor1_position = 0;
 float motor2_position = 0;
 
 SmoothController controller1, controller2;
 
 bool motor1_homed = false;
 bool motor2_homed = false;
 bool system_ready = false;
 
 // Trajectory management
 TrajectoryPoint trajectory_buffer[30];
 int trajectory_count = 0;
 int trajectory_index = 0;
 bool trajectory_active = false;
 unsigned long trajectory_point_start = 0;
 
 // Timing
 unsigned long last_control_update = 0;
 
 // ========== ENCODER INTERRUPTS ==========
 void encoder1_interrupt() {
   bool a_state = digitalRead(ENC1_A);
   bool b_state = digitalRead(ENC1_B);
   
   if (a_state == b_state) {
     encoder1_count++;
   } else {
     encoder1_count--;
   }
 }
 
 void encoder2_interrupt() {
   bool a_state = digitalRead(ENC2_A);
   bool b_state = digitalRead(ENC2_B);
   
   if (a_state == b_state) {
     encoder2_count--;
   } else {
     encoder2_count++;
   }
 }
 
 // ========== MOTOR CONTROL ==========
 void forceMotor1(int speed, int direction) {
   speed = constrain(abs(speed), 0, MAX_SPEED);
   
   digitalWrite(MOT1_DIR1, LOW);
   digitalWrite(MOT1_DIR2, LOW);
   analogWrite(MOT1_PWM, 0);
   delay(5);
   
   if (direction > 0) {
     digitalWrite(MOT1_DIR1, HIGH);
   } else if (direction < 0) {
     digitalWrite(MOT1_DIR2, HIGH);
   }
   
   analogWrite(MOT1_PWM, speed);
 }
 
 void forceMotor2(int speed, int direction) {
   speed = constrain(abs(speed), 0, MAX_SPEED);
   
   digitalWrite(MOT2_DIR1, LOW);
   digitalWrite(MOT2_DIR2, LOW);
   analogWrite(MOT2_PWM, 0);
   delay(5);
   
   if (direction > 0) {
     digitalWrite(MOT2_DIR1, HIGH);
   } else if (direction < 0) {
     digitalWrite(MOT2_DIR2, HIGH);
   }
   
   analogWrite(MOT2_PWM, speed);
 }
 
 void setMotor1(float control_signal) {
   int speed = (int)abs(control_signal);
   int direction = (control_signal > 0) ? 1 : ((control_signal < 0) ? -1 : 0);
   forceMotor1(speed, direction);
 }
 
 void setMotor2(float control_signal) {
   int speed = (int)abs(control_signal);
   int direction = (control_signal > 0) ? 1 : ((control_signal < 0) ? -1 : 0);
   forceMotor2(speed, direction);
 }
 
 void stopAllMotors() {
   forceMotor1(0, 0);
   forceMotor2(0, 0);
   controller1.reset();
   controller2.reset();
   trajectory_active = false;
   Serial.println("⏹ All motors stopped");
 }
 
 // ========== HOMING FUNCTIONS ==========
 bool forceHomeMotor1() {
   Serial.println("🏠 === HOMING MOTOR 1 ===");
   
   if (digitalRead(SENSOR1) == LOW) {
     Serial.println("   Moving away from sensor first...");
     forceMotor1(80, 1);
     delay(1000);
     forceMotor1(0, 0);
     delay(500);
   }
   
   Serial.println("   Starting homing motion...");
   forceMotor1(HOMING_SPEED, -1);
   
   unsigned long start_time = millis();
   while (digitalRead(SENSOR1) == HIGH && (millis() - start_time) < 15000) {
     delay(10);
   }
   
   forceMotor1(0, 0);
   
   if (digitalRead(SENSOR1) == LOW) {
     noInterrupts();
     encoder1_count = 0;
     interrupts();
     motor1_position = 0;
     motor1_homed = true;
     controller1.reset();
     
     Serial.println("✅ Motor 1 homed successfully");
     return true;
   } else {
     Serial.println("❌ Motor 1 homing failed");
     return false;
   }
 }
 
 bool forceHomeMotor2() {
   Serial.println("🏠 === HOMING MOTOR 2 ===");
   
   if (digitalRead(SENSOR2) == LOW) {
     Serial.println("   Moving away from sensor first...");
     forceMotor2(80, 1);
     delay(1000);
     forceMotor2(0, 0);
     delay(500);
   }
   
   Serial.println("   Starting homing motion...");
   forceMotor2(HOMING_SPEED, -1);
   
   unsigned long start_time = millis();
   while (digitalRead(SENSOR2) == HIGH && (millis() - start_time) < 15000) {
     delay(10);
   }
   
   forceMotor2(0, 0);
   
   if (digitalRead(SENSOR2) == LOW) {
     noInterrupts();
     encoder2_count = 0;
     interrupts();
     motor2_position = 0;
     motor2_homed = true;
     controller2.reset();
     
     Serial.println("✅ Motor 2 homed successfully");
     return true;
   } else {
     Serial.println("❌ Motor 2 homing failed");
     return false;
   }
 }
 
 void executeHoming() {
   Serial.println("🚀 === COMPLETE HOMING SEQUENCE ===");
   
   stopAllMotors();
   delay(500);
   
   bool m1_success = forceHomeMotor1();
   delay(1000);
   bool m2_success = forceHomeMotor2();
   
   if (m1_success && m2_success) {
     system_ready = true;
     Serial.println("🎉 BOTH MOTORS HOMED SUCCESSFULLY!");
   } else {
     system_ready = false;
     Serial.println("❌ HOMING FAILED - Check hardware");
   }
   Serial.println("===================================");
 }
 
 // ========== MOTION CONTROL ==========
 void moveToAngles(float angle1, float angle2, float speed_percent = 80) {
   if (!system_ready) {
     Serial.println("❌ System not ready - Please home first");
     return;
   }
   
   controller1.setTarget(angle1);
   controller2.setTarget(angle2);
   
   Serial.println("🎯 === SMOOTH MOVEMENT STARTED ===");
   Serial.print("   Target: θ1=");
   Serial.print(angle1, 2);
   Serial.print("°, θ2=");
   Serial.print(angle2, 2);
   Serial.print("° at ");
   Serial.print(speed_percent, 0);
   Serial.println("% speed");
 }
 
 void updateMotionControl() {
   noInterrupts();
   long enc1 = encoder1_count;
   long enc2 = encoder2_count;
   interrupts();
   
   motor1_position = enc1 / COUNTS_PER_DEGREE;
   motor2_position = enc2 / COUNTS_PER_DEGREE;
   
   float control1 = controller1.updateSmooth(motor1_position);
   float control2 = controller2.updateSmooth(motor2_position);
   
   setMotor1(control1);
   setMotor2(control2);
   
   if (!controller1.isMoving() && !controller2.isMoving() && (controller1.reached_target || controller2.reached_target)) {
     Serial.println("✅ === MOVEMENT COMPLETED ===");
     Serial.print("   Final: θ1=");
     Serial.print(motor1_position, 2);
     Serial.print("°, θ2=");
     Serial.print(motor2_position, 2);
     Serial.println("°");
     Serial.println("=============================");
   }
 }
 
 // ========== TRAJECTORY MANAGEMENT ==========
 void clearTrajectory() {
   trajectory_count = 0;
   trajectory_index = 0;
   trajectory_active = false;
   Serial.println("📝 Trajectory cleared");
 }
 
 void addTrajectoryPoint(float a1, float a2, float speed, unsigned long duration) {
   if (trajectory_count >= 30) {
     Serial.println("❌ Trajectory buffer full!");
     return;
   }
   
   trajectory_buffer[trajectory_count].angle1 = a1;
   trajectory_buffer[trajectory_count].angle2 = a2;
   trajectory_buffer[trajectory_count].speed_percent = speed;
   trajectory_buffer[trajectory_count].duration_ms = duration;
   
   trajectory_count++;
   
   Serial.print("✓ Point ");
   Serial.print(trajectory_count);
   Serial.print(": (");
   Serial.print(a1, 1);
   Serial.print("°, ");
   Serial.print(a2, 1);
   Serial.print("°) ");
   Serial.print(speed, 0);
   Serial.print("% ");
   Serial.print(duration);
   Serial.println("ms");
 }
 
 void startTrajectory() {
   if (trajectory_count == 0) {
     Serial.println("❌ No trajectory points! Use ADD command first");
     Serial.println("💡 Example: ADD 45 30 80 2000");
     return;
   }
   
   if (!system_ready) {
     Serial.println("❌ System not ready - Please HOME first");
     return;
   }
   
   trajectory_active = true;
   trajectory_index = 0;
   trajectory_point_start = millis();
   
   TrajectoryPoint& point = trajectory_buffer[0];
   controller1.setTarget(point.angle1);
   controller2.setTarget(point.angle2);
   
   Serial.println("🚀 === TRAJECTORY STARTED ===");
   Serial.print("   Total points: ");
   Serial.println(trajectory_count);
   Serial.print("   Point 1: (");
   Serial.print(point.angle1, 1);
   Serial.print("°, ");
   Serial.print(point.angle2, 1);
   Serial.println("°)");
 }
 
 void updateTrajectory() {
   if (!trajectory_active) return;
   
   bool point_completed = false;
   
   if (millis() - trajectory_point_start >= trajectory_buffer[trajectory_index].duration_ms) {
     point_completed = true;
   } else if (!controller1.isMoving() && !controller2.isMoving()) {
     point_completed = true;
   }
   
   if (point_completed) {
     trajectory_index++;
     
     if (trajectory_index >= trajectory_count) {
       trajectory_active = false;
       Serial.println("🏁 === TRAJECTORY COMPLETED ===");
       return;
     }
     
     trajectory_point_start = millis();
     TrajectoryPoint& point = trajectory_buffer[trajectory_index];
     controller1.setTarget(point.angle1);
     controller2.setTarget(point.angle2);
     
     Serial.print("📍 Point ");
     Serial.print(trajectory_index + 1);
     Serial.print(": (");
     Serial.print(point.angle1, 1);
     Serial.print("°, ");
     Serial.print(point.angle2, 1);
     Serial.println("°)");
   }
 }
 
 // ========== DEBUG FUNCTIONS ==========
 void testAllPins() {
   Serial.println("🔍 === PIN TEST ===");
   Serial.print("Motor 1 - PWM:");
   Serial.print(MOT1_PWM);
   Serial.print(" DIR1:");
   Serial.print(MOT1_DIR1);
   Serial.print(" DIR2:");
   Serial.println(MOT1_DIR2);
   Serial.print("Motor 2 - PWM:");
   Serial.print(MOT2_PWM);
   Serial.print(" DIR1:");
   Serial.print(MOT2_DIR1);
   Serial.print(" DIR2:");
   Serial.println(MOT2_DIR2);
   Serial.println("===================");
 }
 
 void testMotor1Hardware() {
   Serial.println("🔧 Testing Motor 1...");
   forceMotor1(TEST_SPEED, 1);
   delay(2000);
   forceMotor1(0, 0);
   Serial.println("✅ Motor 1 test done");
 }
 
 void testMotor2Hardware() {
   Serial.println("🔧 Testing Motor 2...");
   forceMotor2(TEST_SPEED, 1);
   delay(2000);
   forceMotor2(0, 0);
   Serial.println("✅ Motor 2 test done");
 }
 
 // ========== COMMAND PROCESSOR ==========
 void processCommand() {
   String cmd = Serial.readString();
   cmd.trim();
   cmd.toUpperCase();
   
   Serial.print("📨 Command: ");
   Serial.println(cmd);
   
   // ========== SYSTEM COMMANDS ==========
   if (cmd == "HOME" || cmd == "INIT") {
     executeHoming();
   }
   else if (cmd == "POS") {
     Serial.println("📍 === POSITIONS ===");
     Serial.print("Motor 1: ");
     Serial.print(motor1_position, 2);
     Serial.print("° (");
     Serial.print(motor1_homed ? "✅" : "❌");
     Serial.println(" Homed)");
     Serial.print("Motor 2: ");
     Serial.print(motor2_position, 2);
     Serial.print("° (");
     Serial.print(motor2_homed ? "✅" : "❌");
     Serial.println(" Homed)");
     Serial.println("====================");
   }
   else if (cmd == "STOP") {
     stopAllMotors();
   }
   else if (cmd == "STATUS") {
     Serial.println("📊 === STATUS ===");
     Serial.print("System Ready: ");
     Serial.println(system_ready ? "✅ YES" : "❌ NO");
     Serial.print("Motors Homed: M1=");
     Serial.print(motor1_homed ? "✅" : "❌");
     Serial.print(" M2=");
     Serial.println(motor2_homed ? "✅" : "❌");
     Serial.print("Trajectory Points: ");
     Serial.println(trajectory_count);
     Serial.println("=================");
   }
   
   // ========== MOTION COMMANDS ==========
   else if (cmd.startsWith("MOVE ")) {
     int space1 = cmd.indexOf(' ');
     int space2 = cmd.indexOf(' ', space1 + 1);
     int space3 = cmd.indexOf(' ', space2 + 1);
     
     if (space1 > 0 && space2 > space1) {
       float a1 = cmd.substring(space1 + 1, space2).toFloat();
       float a2, speed = 80;
       
       if (space3 > space2) {
         a2 = cmd.substring(space2 + 1, space3).toFloat();
         speed = cmd.substring(space3 + 1).toFloat();
       } else {
         a2 = cmd.substring(space2 + 1).toFloat();
       }
       
       moveToAngles(a1, a2, speed);
     } else {
       Serial.println("📝 Format: MOVE angle1 angle2 [speed]");
       Serial.println("   Example: MOVE 45 30 80");
     }
   }
   
   // ========== TRAJECTORY COMMANDS ==========
   else if (cmd.startsWith("ADD ")) {
     String params = cmd.substring(4);
     int s1 = params.indexOf(' ');
     int s2 = params.indexOf(' ', s1 + 1);
     int s3 = params.indexOf(' ', s2 + 1);
     
     if (s1 > 0 && s2 > s1 && s3 > s2) {
       float a1 = params.substring(0, s1).toFloat();
       float a2 = params.substring(s1 + 1, s2).toFloat();
       float speed = params.substring(s2 + 1, s3).toFloat();
       unsigned long duration = params.substring(s3 + 1).toInt();
       
       addTrajectoryPoint(a1, a2, speed, duration);
     } else {
       Serial.println("📝 Format: ADD angle1 angle2 speed duration");
       Serial.println("   Example: ADD 45 30 80 2000");
     }
   }
   else if (cmd == "START") {
     startTrajectory();
   }
   else if (cmd == "CLEAR") {
     clearTrajectory();
   }
   else if (cmd == "DEMO") {
     Serial.println("🎭 Loading demo trajectory...");
     clearTrajectory();
     addTrajectoryPoint(0, 0, 70, 1500);
     addTrajectoryPoint(45, 30, 80, 2500);
     addTrajectoryPoint(-30, 60, 75, 2000);
     addTrajectoryPoint(0, 0, 65, 2000);
     Serial.println("✅ Demo loaded - Type START");
   }
   
   // ========== DEBUG COMMANDS ==========
   else if (cmd == "TEST_PINS") {
     testAllPins();
   }
   else if (cmd == "TEST_M1") {
     testMotor1Hardware();
   }
   else if (cmd == "TEST_M2") {
     testMotor2Hardware();
   }
   else if (cmd == "M1_FWD") {
     forceMotor1(TEST_SPEED, 1);
     delay(2000);
     forceMotor1(0, 0);
   }
   else if (cmd == "M1_REV") {
     forceMotor1(TEST_SPEED, -1);
     delay(2000);
     forceMotor1(0, 0);
   }
   else if (cmd == "M2_FWD") {
     forceMotor2(TEST_SPEED, 1);
     delay(2000);
     forceMotor2(0, 0);
   }
   else if (cmd == "M2_REV") {
     forceMotor2(TEST_SPEED, -1);
     delay(2000);
     forceMotor2(0, 0);
   }
   else if (cmd == "TEST_SENSORS") {
     Serial.println("🔍 === SENSORS ===");
     Serial.print("Sensor 1: ");
     Serial.println(digitalRead(SENSOR1) == HIGH ? "CLEAR" : "BLOCKED");
     Serial.print("Sensor 2: ");
     Serial.println(digitalRead(SENSOR2) == HIGH ? "CLEAR" : "BLOCKED");
     Serial.println("==================");
   }
   
   // ========== HELP ==========
   else if (cmd == "HELP") {
     Serial.println("🎓 === ALL COMMANDS ===");
     Serial.println("🏠 SYSTEM:");
     Serial.println("   HOME/INIT      - Home both motors");
     Serial.println("   POS            - Show positions");
     Serial.println("   STATUS         - System status");
     Serial.println("   STOP           - Stop all");
     Serial.println();
     Serial.println("🎯 MOTION:");
     Serial.println("   MOVE a1 a2 s   - Move to angles");
     Serial.println("   Example: MOVE 45 30 80");
     Serial.println();
     Serial.println("📊 TRAJECTORY:");
     Serial.println("   ADD a1 a2 s d  - Add trajectory point");
     Serial.println("   START          - Execute trajectory");
     Serial.println("   CLEAR          - Clear trajectory");
     Serial.println("   DEMO           - Load demo");
     Serial.println();
     Serial.println("🔧 DEBUG:");
     Serial.println("   TEST_M1        - Test motor 1");
     Serial.println("   TEST_M2        - Test motor 2");
     Serial.println("   M1_FWD/M1_REV  - Manual motor 1");
     Serial.println("   M2_FWD/M2_REV  - Manual motor 2");
     Serial.println("   TEST_SENSORS   - Test sensors");
     Serial.println("   TEST_PINS      - Test pin config");
     Serial.println("=======================");
   }
   else {
     Serial.println("❓ Unknown command");
     Serial.println("💡 Type HELP for all commands");
   }
   
   Serial.println();
 }
 
 // ========== SETUP ==========
 void setup() {
   Serial.begin(115200);
   delay(1000);
   
   Serial.println("╔══════════════════════════════════════════════════════╗");
   Serial.println("║          COMPLETE DEBUG + NORMAL COMMANDS           ║");
   Serial.println("║              ALL FUNCTIONS INCLUDED                 ║");
   Serial.println("╚══════════════════════════════════════════════════════╝");
   Serial.println();
   
   pinMode(ENC1_A, INPUT_PULLUP);
   pinMode(ENC1_B, INPUT_PULLUP);
   pinMode(ENC2_A, INPUT_PULLUP);
   pinMode(ENC2_B, INPUT_PULLUP);
   
   pinMode(MOT1_PWM, OUTPUT);
   pinMode(MOT1_DIR1, OUTPUT);
   pinMode(MOT1_DIR2, OUTPUT);
   pinMode(MOT2_PWM, OUTPUT);
   pinMode(MOT2_DIR1, OUTPUT);
   pinMode(MOT2_DIR2, OUTPUT);
   
   pinMode(SENSOR1, INPUT_PULLUP);
   pinMode(SENSOR2, INPUT_PULLUP);
   pinMode(ESTOP_PIN, INPUT_PULLUP);
   
   attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1_interrupt, CHANGE);
   attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2_interrupt, CHANGE);
   
   controller1.reset();
   controller2.reset();
   clearTrajectory();
   stopAllMotors();
   
   Serial.println("✅ All systems initialized");
   Serial.println();
   Serial.println("🚀 QUICK START:");
   Serial.println("   HOME           - Home both motors");
   Serial.println("   TEST_M2        - Test if motor 2 works");
   Serial.println("   DEMO           - Load demo trajectory");
   Serial.println("   START          - Execute trajectory");
   Serial.println("   HELP           - All commands");
   Serial.println();
 }
 
 // ========== MAIN LOOP ==========
 void loop() {
   if (millis() - last_control_update >= 20) {
     updateMotionControl();
     updateTrajectory();
     last_control_update = millis();
   }
   
   if (Serial.available()) {
     processCommand();
   }
 }
 
