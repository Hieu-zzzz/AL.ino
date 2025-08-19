/*
 * ============================================================================
 * HỆ THỐNG ĐIỀU KHIỂN DC MOTOR NÂNG CAO - SEQUENCE EXECUTOR V4.0 - FIXED
 * ============================================================================
 * 
 * TÍNH NĂNG MỚI:
 * - Advanced command parsing với nhiều tham số
 * - Sequence execution system
 * - Batch command processing
 * - Auto delay và speed control
 * - Bug fixes và optimization
 * ============================================================================
 */

// ========== CẤU HÌNH HARDWARE ==========
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_ENA 5
#define MOTOR_IN1 30
#define MOTOR_IN2 31
#define LIMIT_MIN 10
#define LIMIT_MAX 11
#define EMERGENCY_STOP 12

// ========== THÔNG SỐ KỸ THUẬT ==========
#define ENCODER_PPR 500
#define SCREW_PITCH 10.0
#define GEAR_RATIO 1.0
const float PULSES_PER_MM = (ENCODER_PPR * GEAR_RATIO) / SCREW_PITCH;
const float MM_PER_PULSE = SCREW_PITCH / (ENCODER_PPR * GEAR_RATIO);
const float THEORETICAL_ACCURACY = MM_PER_PULSE * 1000;

#define MAX_TRAVEL_MM 1000.0
#define MIN_TRAVEL_MM -1000.0
#define POSITION_TOLERANCE_MM 0.05
#define BACKLASH_COMPENSATION 0.1
#define MAX_VELOCITY_MM_PER_SEC 50.0

#define MIN_MOTOR_SPEED 25
#define MAX_MOTOR_SPEED 255
#define APPROACH_SPEED 80
#define FINAL_SPEED 30
#define DEFAULT_SPEED 127

// ========== HỆ THỐNG SEQUENCE ==========
#define MAX_SEQUENCE_STEPS 50
#define MAX_COMMAND_LENGTH 128

// ========== KHAI BÁO BIẾN GLOBAL (SỬA LỖI) ==========
volatile long encoder_count = 0;
int encoder_direction = 1;
bool backlash_compensation_enabled = true;
int last_movement_direction = 0;

// CÁC BIẾN CONTROL MODE (THÊM VÀO ĐỂ SỬA LỖI)
bool position_control_mode = false;  // *** SỬA LỖI: THÊM BIẾN NÀY ***
bool continuous_display = true;
bool detailed_mode = false;
unsigned long last_display_time = 0;
const unsigned long DISPLAY_INTERVAL = 200;

// ========== CẤU TRÚC PID CONTROLLER ==========
struct PIDController {
  float Kp = 0.5, Ki = 0.5, Kd = 1.0;
  float previous_error = 0.0, integral = 0.0;
  unsigned long last_time = 0;
  
  void reset() {
    previous_error = 0.0;
    integral = 0.0;
    last_time = millis();
  }
  
  float calculate(float error, float dt) {
    if (dt <= 0) return 0;
    float p_term = Kp * error;
    integral += error * dt;
    integral = constrain(integral, -100.0, 100.0);
    float i_term = Ki * integral;
    float derivative = (error - previous_error) / dt;
    float d_term = Kd * derivative;
    previous_error = error;
    return p_term + i_term + d_term;
  }
};

// ========== CẤU TRÚC PRECISION CONTROL ==========
struct PrecisionControl {
  long position_pulses = 0;
  float position_mm = 0.0;
  float target_position_mm = 0.0;
  long target_pulses = 0;
  
  float velocity_mm_per_sec = 0.0;
  float acceleration_mm_per_sec2 = 0.0;
  float rpm = 0.0;
  
  int motor_speed = 0;
  int motor_direction = 0;
  bool is_moving = false;
  int jog_speed = DEFAULT_SPEED;
  
  float position_error_mm = 0.0;
  float max_error_recorded = 0.0;
  
  float total_distance_traveled = 0.0;
  float max_position_reached = 0.0;
  float min_position_reached = 0.0;
  float max_velocity_achieved = 0.0;
  
  unsigned long last_update_time = 0;
  unsigned long start_time = 0;
  long last_position = 0;
  float last_velocity = 0.0;
  unsigned long move_start_time = 0;
  
  uint32_t successful_moves = 0;
  uint32_t failed_moves = 0;
  
  bool limits_enabled = true;
  bool emergency_stop_active = false;
  bool homing_mode = false;
};

// ========== CẤU TRÚC SEQUENCE STEP ==========
struct SequenceStep {
  String command_type;     // MOVE, GOTO, PULSES, DELAY, SPEED
  float parameter1;        // distance/position/pulses
  int parameter2;          // speed (optional)
  unsigned long parameter3; // delay time (optional)
  bool completed = false;
};

// ========== CẤU TRÚC SEQUENCE EXECUTOR ==========
struct SequenceExecutor {
  SequenceStep steps[MAX_SEQUENCE_STEPS];
  int total_steps = 0;
  int current_step = 0;
  bool is_running = false;
  bool is_paused = false;
  bool auto_continue = true;
  
  // Timing
  unsigned long step_start_time = 0;
  unsigned long delay_start_time = 0;
  bool in_delay = false;
  
  // Statistics
  int completed_steps = 0;
  int failed_steps = 0;
  unsigned long total_execution_time = 0;
  
  void reset() {
    total_steps = 0;
    current_step = 0;
    is_running = false;
    is_paused = false;
    completed_steps = 0;
    failed_steps = 0;
    in_delay = false;
    
    for(int i = 0; i < MAX_SEQUENCE_STEPS; i++) {
      steps[i].completed = false;
    }
  }
  
  bool addStep(String cmd_type, float p1, int p2 = 0, unsigned long p3 = 0) {
    if(total_steps >= MAX_SEQUENCE_STEPS) {
      Serial.println("✗ Sequence full! Max 50 steps");
      return false;
    }
    
    steps[total_steps].command_type = cmd_type;
    steps[total_steps].parameter1 = p1;
    steps[total_steps].parameter2 = p2;
    steps[total_steps].parameter3 = p3;
    steps[total_steps].completed = false;
    total_steps++;
    
    return true;
  }
  
  void showSequence() {
    Serial.println("=== CURRENT SEQUENCE ===");
    if(total_steps == 0) {
      Serial.println("No steps defined");
      return;
    }
    
    for(int i = 0; i < total_steps; i++) {
      Serial.print(i == current_step && is_running ? "-> " : "   ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(steps[i].command_type);
      Serial.print(" ");
      Serial.print(steps[i].parameter1, 2);
      
      if(steps[i].parameter2 > 0) {
        Serial.print(" S:");
        Serial.print(steps[i].parameter2);
      }
      
      if(steps[i].parameter3 > 0) {
        Serial.print(" D:");
        Serial.print(steps[i].parameter3);
        Serial.print("ms");
      }
      
      if(steps[i].completed) {
        Serial.print(" ✓");
      }
      
      Serial.println();
    }
    
    Serial.print("Status: ");
    if(is_running && is_paused) Serial.println("PAUSED");
    else if(is_running) Serial.println("RUNNING");
    else Serial.println("STOPPED");
    
    Serial.print("Progress: ");
    Serial.print(completed_steps);
    Serial.print("/");
    Serial.println(total_steps);
    Serial.println("========================");
  }
  
  void start() {
    if(total_steps == 0) {
      Serial.println("✗ No sequence to run!");
      return;
    }
    
    current_step = 0;
    is_running = true;
    is_paused = false;
    step_start_time = millis();
    completed_steps = 0;  // Reset completed steps
    
    Serial.print("✓ Starting sequence with ");
    Serial.print(total_steps);
    Serial.println(" steps...");
    
    executeCurrentStep();
  }
  
  void pause() {
    is_paused = true;
    Serial.println("⏸ Sequence paused");
  }
  
  void resume() {
    is_paused = false;
    Serial.println("▶ Sequence resumed");
  }
  
  void stop() {
    is_running = false;
    is_paused = false;
    in_delay = false;
    Serial.println("⏹ Sequence stopped");
  }
  
  void executeCurrentStep();  // Forward declaration
  void update();             // Forward declaration
  
private:
  void executeMove(float distance, int speed);
  void executeGoto(float position, int speed);
  void executePulses(long pulses, int speed);
  void executeDelay(unsigned long delay_ms);
  void executeSpeed(int speed);
  void moveToNextStep();
};

// ========== KHAI BÁO BIẾN CHÍNH ==========
PrecisionControl precision;
PIDController pid_controller;
SequenceExecutor sequence;

// ========== FORWARD DECLARATIONS (SỬA LỖI) ==========
void setMotorSpeed(int speed, int direction);
void stopMotor();
void updateStatistics();
void executePositionControl();

// ========== ENCODER INTERRUPT ==========
void encoderISR() {
  static unsigned long last_interrupt = 0;
  static bool last_a = false;
  
  unsigned long current_time = micros();
  if (current_time - last_interrupt < 25) return;
  
  bool current_a = digitalRead(ENCODER_A);
  bool current_b = digitalRead(ENCODER_B);
  
  if (current_a != last_a) {
    if (current_a == current_b) {
      encoder_count -= encoder_direction;
    } else {
      encoder_count += encoder_direction;
    }
  }
  
  last_a = current_a;
  last_interrupt = current_time;
}

// ========== MOTOR CONTROL FUNCTIONS ==========
void setMotorSpeed(int speed, int direction) {
  if (precision.emergency_stop_active) {
    stopMotor();
    return;
  }
  
  if (precision.limits_enabled) {
    if (direction > 0 && digitalRead(LIMIT_MAX) == LOW) {
      stopMotor();
      Serial.println("⚠ MAX limit switch hit!");
      return;
    }
    if (direction < 0 && digitalRead(LIMIT_MIN) == LOW) {
      stopMotor();
      Serial.println("⚠ MIN limit switch hit!");
      return;
    }
  }
  
  speed = constrain(abs(speed), 0, MAX_MOTOR_SPEED);
  
  if (direction > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    precision.motor_direction = 1;
  } else if (direction < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    precision.motor_direction = -1;
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    precision.motor_direction = 0;
  }
  
  analogWrite(MOTOR_ENA, speed);
  precision.motor_speed = speed;
  precision.is_moving = (speed > 0);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
  
  precision.motor_speed = 0;
  precision.motor_direction = 0;
  precision.is_moving = false;
  
  pid_controller.reset();
}

void emergencyStop() {
  stopMotor();
  position_control_mode = false;
  precision.emergency_stop_active = true;
  precision.homing_mode = false;
  sequence.stop();
  
  Serial.println("🚨 EMERGENCY STOP ACTIVATED! 🚨");
  Serial.println("Send 'RESET' to clear emergency stop.");
}

// ========== SEQUENCE EXECUTOR METHODS IMPLEMENTATION ==========
void SequenceExecutor::executeCurrentStep() {
  if(!is_running || is_paused || current_step >= total_steps) return;
  
  SequenceStep& step = steps[current_step];
  
  Serial.print("Executing step ");
  Serial.print(current_step + 1);
  Serial.print("/");
  Serial.print(total_steps);
  Serial.print(": ");
  Serial.print(step.command_type);
  Serial.print(" ");
  Serial.println(step.parameter1, 2);
  
  if(step.command_type == "MOVE") {
    executeMove(step.parameter1, step.parameter2);
  }
  else if(step.command_type == "GOTO") {
    executeGoto(step.parameter1, step.parameter2);
  }
  else if(step.command_type == "PULSES") {
    executePulses((long)step.parameter1, step.parameter2);
  }
  else if(step.command_type == "DELAY") {
    executeDelay(step.parameter3);
  }
  else if(step.command_type == "SPEED") {
    executeSpeed(step.parameter2);
  }
}

void SequenceExecutor::update() {
  if(!is_running || is_paused) return;
  
  // Handle delay step
  if(in_delay) {
    if(millis() - delay_start_time >= steps[current_step].parameter3) {
      in_delay = false;
      moveToNextStep();
    } else {
      // Show countdown occasionally
      static unsigned long last_countdown = 0;
      if(millis() - last_countdown > 1000) {
        unsigned long remaining = steps[current_step].parameter3 - (millis() - delay_start_time);
        Serial.print("Delay remaining: ");
        Serial.print(remaining / 1000);
        Serial.println("s");
        last_countdown = millis();
      }
    }
    return;
  }
  
  // Check if current movement step is completed
  if(!position_control_mode && !precision.homing_mode) {
    // Current step finished
    moveToNextStep();
  }
}

void SequenceExecutor::executeMove(float distance, int speed) {
  if(speed > 0) {
    precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  float new_target = precision.position_mm + distance;
  if(new_target < MIN_TRAVEL_MM || new_target > MAX_TRAVEL_MM) {
    Serial.println("✗ Move would exceed limits - skipping step");
    moveToNextStep();
    return;
  }
  
  precision.target_position_mm = new_target;
  precision.target_pulses = new_target * PULSES_PER_MM;
  position_control_mode = true;
  precision.homing_mode = false;
  precision.move_start_time = millis();
  pid_controller.reset();
}

void SequenceExecutor::executeGoto(float position, int speed) {
  if(speed > 0) {
    precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  if(position < MIN_TRAVEL_MM || position > MAX_TRAVEL_MM) {
    Serial.println("✗ Position out of range - skipping step");
    moveToNextStep();
    return;
  }
  
  precision.target_position_mm = position;
  precision.target_pulses = position * PULSES_PER_MM;
  position_control_mode = true;
  precision.homing_mode = false;
  precision.move_start_time = millis();
  pid_controller.reset();
}

void SequenceExecutor::executePulses(long pulses, int speed) {
  if(speed > 0) {
    precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  float distance_mm = pulses * MM_PER_PULSE;
  float new_target = precision.position_mm + distance_mm;
  
  if(new_target < MIN_TRAVEL_MM || new_target > MAX_TRAVEL_MM) {
    Serial.println("✗ Pulses would exceed limits - skipping step");
    moveToNextStep();
    return;
  }
  
  precision.target_position_mm = new_target;
  precision.target_pulses = precision.position_pulses + pulses;
  position_control_mode = true;
  precision.homing_mode = false;
  precision.move_start_time = millis();
  pid_controller.reset();
}

void SequenceExecutor::executeDelay(unsigned long delay_ms) {
  in_delay = true;
  delay_start_time = millis();
  Serial.print("Delaying for ");
  Serial.print(delay_ms / 1000.0, 1);
  Serial.println("s");
}

void SequenceExecutor::executeSpeed(int speed) {
  precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  Serial.print("Speed set to ");
  Serial.println(speed);
  moveToNextStep(); // Speed change is instant
}

void SequenceExecutor::moveToNextStep() {
  steps[current_step].completed = true;
  completed_steps++;
  current_step++;
  
  if(current_step >= total_steps) {
    // Sequence completed
    is_running = false;
    unsigned long total_time = millis() - step_start_time;
    
    Serial.println("🎉 SEQUENCE COMPLETED!");
    Serial.print("Total time: ");
    Serial.print(total_time / 1000.0, 1);
    Serial.println("s");
    Serial.print("Steps completed: ");
    Serial.print(completed_steps);
    Serial.print("/");
    Serial.println(total_steps);
    
    return;
  }
  
  // Execute next step
  if(auto_continue) {
    delay(100); // Small delay between steps
    executeCurrentStep();
  }
}

// ========== PRECISION CALCULATION ENGINE ==========
void updatePrecisionData() {
  unsigned long current_time = millis();
  
  if (digitalRead(EMERGENCY_STOP) == LOW) {
    if (!precision.emergency_stop_active) {
      emergencyStop();
    }
    return;
  }
  
  noInterrupts();
  long current_position = encoder_count;
  interrupts();
  
  precision.position_pulses = current_position;
  precision.position_mm = current_position * MM_PER_PULSE;
  
  if (current_time - precision.last_update_time >= 50) {
    long position_change = current_position - precision.last_position;
    float time_delta = (current_time - precision.last_update_time) / 1000.0;
    
    if (time_delta > 0) {
      float new_velocity = (position_change * MM_PER_PULSE) / time_delta;
      precision.velocity_mm_per_sec = (precision.velocity_mm_per_sec * 0.7) + (new_velocity * 0.3);
      precision.rpm = (position_change / (float)ENCODER_PPR) * (60.0 / time_delta);
      
      float velocity_change = precision.velocity_mm_per_sec - precision.last_velocity;
      precision.acceleration_mm_per_sec2 = velocity_change / time_delta;
      precision.last_velocity = precision.velocity_mm_per_sec;
    }
    
    precision.last_position = current_position;
    precision.last_update_time = current_time;
  }
  
  updateStatistics();
  
  if (position_control_mode && !precision.emergency_stop_active) {
    executePositionControl();
  }
  
  // Update sequence
  sequence.update();
}

void updateStatistics() {
  if (precision.position_mm > precision.max_position_reached) {
    precision.max_position_reached = precision.position_mm;
  }
  if (precision.position_mm < precision.min_position_reached) {
    precision.min_position_reached = precision.position_mm;
  }
  
  if (abs(precision.velocity_mm_per_sec) > precision.max_velocity_achieved) {
    precision.max_velocity_achieved = abs(precision.velocity_mm_per_sec);
  }
  
  if (position_control_mode) {
    precision.position_error_mm = precision.target_position_mm - precision.position_mm;
    
    if (abs(precision.position_error_mm) > precision.max_error_recorded) {
      precision.max_error_recorded = abs(precision.position_error_mm);
    }
  }
}

void executePositionControl() {
  float error = precision.position_error_mm;
  unsigned long current_time = millis();
  
  if (abs(error) <= POSITION_TOLERANCE_MM) {
    stopMotor();
    position_control_mode = false;
    precision.successful_moves++;
    
    unsigned long move_time = current_time - precision.move_start_time;
    
    Serial.print("✓ Position reached: ");
    Serial.print(precision.position_mm, 3);
    Serial.print("mm (Error: ");
    Serial.print(error * 1000, 1);
    Serial.print("µm, Time: ");
    Serial.print(move_time / 1000.0, 1);
    Serial.println("s)");
    return;
  }
  
  if (current_time - precision.move_start_time > 30000) {
    stopMotor();
    position_control_mode = false;
    precision.failed_moves++;
    
    Serial.print("✗ Move timeout! Position: ");
    Serial.print(precision.position_mm, 3);
    Serial.print("mm, Error: ");
    Serial.print(error * 1000, 1);
    Serial.println("µm");
    return;
  }
  
  float compensated_error = error;
  if (backlash_compensation_enabled) {
    int current_direction = (error > 0) ? 1 : -1;
    if (last_movement_direction != 0 && last_movement_direction != current_direction) {
      float compensation = BACKLASH_COMPENSATION * current_direction;
      precision.target_position_mm += compensation;
      precision.target_pulses = precision.target_position_mm * PULSES_PER_MM;
      compensated_error = precision.target_position_mm - precision.position_mm;
    }
    last_movement_direction = current_direction;
  }
  
  float dt = (current_time - pid_controller.last_time) / 1000.0;
  if (dt >= 0.01) {
    float pid_output = pid_controller.calculate(compensated_error, dt);
    pid_controller.last_time = current_time;
    
    int direction = (pid_output > 0) ? 1 : -1;
    int speed;
    
    float abs_error = abs(compensated_error);
    if (abs_error > 5.0) {
      speed = constrain(abs(pid_output * 50), MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    } else if (abs_error > 1.0) {
      speed = APPROACH_SPEED;
    } else {
      speed = FINAL_SPEED;
    }
    
    setMotorSpeed(speed, direction);
  }
}

// ========== SETUP & LOOP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LIMIT_MIN, INPUT_PULLUP);
  pinMode(LIMIT_MAX, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);
  
  stopMotor();
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);
  
  precision.start_time = millis();
  precision.last_update_time = millis();
  pid_controller.reset();
  sequence.reset();
  
  printWelcomeMessage();
}

void loop() {
  updatePrecisionData();
  
  if (continuous_display && millis() - last_display_time >= DISPLAY_INTERVAL) {
    displayPrecisionData();
    last_display_time = millis();
  }
  
  if (Serial.available()) {
    String command = Serial.readString();
    processAdvancedCommand(command);
  }
  
  delay(1);
}

// ========== COMMAND PROCESSING ==========
void processAdvancedCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  // Parse multiple parameters
  int param_count = 0;
  String params[10];  // Max 10 parameters
  
  int start = 0;
  int space_pos = 0;
  
  // Split command by spaces
  while ((space_pos = cmd.indexOf(' ', start)) != -1 && param_count < 10) {
    params[param_count] = cmd.substring(start, space_pos);
    start = space_pos + 1;
    param_count++;
  }
  if (start < cmd.length() && param_count < 10) {
    params[param_count] = cmd.substring(start);
    param_count++;
  }
  
  if (param_count == 0) return;
  
  String command = params[0];
  
  // ADVANCED SEQUENCE COMMANDS
  if (command == "RUN") {
    parseRunCommand(params, param_count);
  }
  else if (command == "SEQ_ADD") {
    parseSequenceAdd(params, param_count);
  }
  else if (command == "SEQ_START") {
    sequence.start();
  }
  else if (command == "SEQ_PAUSE") {
    sequence.pause();
  }
  else if (command == "SEQ_RESUME") {
    sequence.resume();
  }
  else if (command == "SEQ_STOP") {
    sequence.stop();
  }
  else if (command == "SEQ_SHOW") {
    sequence.showSequence();
  }
  else if (command == "SEQ_CLEAR") {
    sequence.reset();
    Serial.println("✓ Sequence cleared");
  }
  else if (command == "COMPLEX") {
    parseComplexCommand(params, param_count);
  }
  
  // TRADITIONAL COMMANDS
  else if (command == "HELP" || command == "H") {
    printAdvancedHelp();
  }
  else if (command == "RESET" || command == "R") {
    resetSystem();
  }
  else if (command == "ZERO" || command == "Z") {
    zeroPosition();
  }
  else if (command == "STATUS" || command == "S") {
    showCurrentStatus();
  }
  else if (command == "INFO" || command == "I") {
    showSystemInfo();
  }
  else if (command == "START") {
    startContinuousDisplay();
  }
  else if (command == "STOP") {
    stopContinuousDisplay();
  }
  else if (command == "DETAILED" || command == "D") {
    toggleDetailedMode();
  }
  else if (command.startsWith("GOTO") && param_count >= 2) {
    parseGotoCommand(params, param_count);
  }
  else if (command.startsWith("MOVE") && param_count >= 2) {
    parseMoveCommand(params, param_count);
  }
  else if (command.startsWith("PULSES") && param_count >= 2) {
    parsePulsesCommand(params, param_count);
  }
  else if (command.startsWith("JOG_")) {
    parseJogCommand(cmd);
  }
  else if (command == "MOTOR_STOP") {
    stopMotor();
    position_control_mode = false;
    precision.homing_mode = false;
    Serial.println("✓ Motor stopped");
  }
  else if (command == "BACKLASH_ON") {
    backlash_compensation_enabled = true;
    Serial.println("✓ Backlash compensation enabled");
  }
  else if (command == "BACKLASH_OFF") {
    backlash_compensation_enabled = false;
    Serial.println("✓ Backlash compensation disabled");
  }
  else if (command == "LIMITS_ON") {
    precision.limits_enabled = true;
    Serial.println("✓ Limit switches enabled");
  }
  else if (command == "LIMITS_OFF") {
    precision.limits_enabled = false;
    Serial.println("⚠ Limit switches disabled");
  }
  else if (command.length() > 0) {
    Serial.println("Unknown command! Type HELP for available commands.");
  }
}

// ========== PARSING FUNCTIONS ==========
void parseRunCommand(String params[], int count) {
  if (count < 3) {
    Serial.println("Format: RUN <MOVE/GOTO/PULSES> <value> [speed] [delay_ms]");
    return;
  }
  
  if (precision.emergency_stop_active) {
    Serial.println("✗ Emergency stop active! Send RESET first.");
    return;
  }
  
  String action = params[1];
  float value = params[2].toFloat();
  int speed = (count > 3) ? params[3].toInt() : 0;
  unsigned long delay_ms = (count > 4) ? params[4].toInt() : 0;
  
  Serial.println("=== EXECUTING RUN COMMAND ===");
  Serial.print("Action: ");
  Serial.print(action);
  Serial.print(" ");
  Serial.print(value);
  
  if (speed > 0) {
    Serial.print(" at speed ");
    Serial.print(speed);
  }
  if (delay_ms > 0) {
    Serial.print(", then delay ");
    Serial.print(delay_ms / 1000.0, 1);
    Serial.print("s");
  }
  Serial.println();
  
  sequence.reset();
  
  if (action == "MOVE") {
    sequence.addStep("MOVE", value, speed, 0);
  } else if (action == "GOTO") {
    sequence.addStep("GOTO", value, speed, 0);
  } else if (action == "PULSES") {
    sequence.addStep("PULSES", value, speed, 0);
  } else {
    Serial.println("✗ Invalid action! Use MOVE, GOTO, or PULSES");
    return;
  }
  
  if (delay_ms > 0) {
    sequence.addStep("DELAY", 0, 0, delay_ms);
  }
  
  sequence.start();
}

void parseComplexCommand(String params[], int count) {
  Serial.println("=== PARSING COMPLEX SEQUENCE ===");
  
  sequence.reset();
  
  int i = 1;
  while (i < count) {
    if (i + 1 >= count) break;
    
    String action = params[i];
    float value = params[i + 1].toFloat();
    int speed = (i + 2 < count && params[i + 2].toFloat() > 20) ? params[i + 2].toInt() : 0;
    unsigned long delay_ms = 0;
    
    int param_used = 2;  // action + value
    if (speed > 0) {
      param_used = 3;
      if (i + 3 < count && params[i + 3].toFloat() > 100) {
        delay_ms = params[i + 3].toInt();
        param_used = 4;
      }
    }
    
    if (action == "MOVE" || action == "GOTO" || action == "PULSES") {
      sequence.addStep(action, value, speed, 0);
      
      if (delay_ms > 0) {
        sequence.addStep("DELAY", 0, 0, delay_ms);
      }
      
      Serial.print("Added: ");
      Serial.print(action);
      Serial.print(" ");
      Serial.print(value);
      if (speed > 0) {
        Serial.print(" S:");
        Serial.print(speed);
      }
      if (delay_ms > 0) {
        Serial.print(" + Delay ");
        Serial.print(delay_ms / 1000.0, 1);
        Serial.print("s");
      }
      Serial.println();
    }
    
    i += param_used;
  }
  
  Serial.print("Total steps created: ");
  Serial.println(sequence.total_steps);
  
  if (sequence.total_steps > 0) {
    sequence.start();
  } else {
    Serial.println("No valid steps found!");
  }
}

void parseSequenceAdd(String params[], int count) {
  if (count < 3) {
    Serial.println("Format: SEQ_ADD <MOVE/GOTO/PULSES/DELAY/SPEED> <value> [speed] [delay_ms]");
    return;
  }
  
  String action = params[1];
  float value = params[2].toFloat();
  int speed = (count > 3) ? params[3].toInt() : 0;
  unsigned long delay_ms = (count > 4) ? params[4].toInt() : 0;
  
  if (action == "DELAY") {
    sequence.addStep("DELAY", 0, 0, (unsigned long)value);
    Serial.print("✓ Added delay: ");
    Serial.print(value / 1000.0, 1);
    Serial.println("s");
  } else if (action == "SPEED") {
    sequence.addStep("SPEED", 0, (int)value, 0);
    Serial.print("✓ Added speed change: ");
    Serial.println((int)value);
  } else if (action == "MOVE" || action == "GOTO" || action == "PULSES") {
    sequence.addStep(action, value, speed, delay_ms);
    Serial.print("✓ Added step: ");
    Serial.print(action);
    Serial.print(" ");
    Serial.print(value);
    if (speed > 0) Serial.print(" S:" + String(speed));
    if (delay_ms > 0) Serial.print(" D:" + String(delay_ms) + "ms");
    Serial.println();
  } else {
    Serial.println("✗ Invalid action!");
  }
}

// ========== BASIC COMMAND PARSING ==========
void parseGotoCommand(String params[], int count) {
  if (precision.emergency_stop_active) {
    Serial.println("✗ Emergency stop active! Send RESET first.");
    return;
  }
  
  float target = params[1].toFloat();
  int speed = (count > 2) ? params[2].toInt() : 0;
  
  if (target < MIN_TRAVEL_MM || target > MAX_TRAVEL_MM) {
    Serial.print("✗ Position out of range! Valid range: ");
    Serial.print(MIN_TRAVEL_MM);
    Serial.print(" to ");
    Serial.print(MAX_TRAVEL_MM);
    Serial.println("mm");
    return;
  }
  
  if (speed > 0) {
    precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  precision.target_position_mm = target;
  precision.target_pulses = target * PULSES_PER_MM;
  position_control_mode = true;
  precision.homing_mode = false;
  precision.move_start_time = millis();
  pid_controller.reset();
  
  Serial.print("✓ Moving to position: ");
  Serial.print(target, 3);
  Serial.println("mm");
}

void parseMoveCommand(String params[], int count) {
  if (precision.emergency_stop_active) {
    Serial.println("✗ Emergency stop active! Send RESET first.");
    return;
  }
  
  float distance = params[1].toFloat();
  int speed = (count > 2) ? params[2].toInt() : 0;
  float new_target = precision.position_mm + distance;
  
  if (new_target < MIN_TRAVEL_MM || new_target > MAX_TRAVEL_MM) {
    Serial.println("✗ Move would exceed travel limits!");
    return;
  }
  
  if (speed > 0) {
    precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  precision.target_position_mm = new_target;
  precision.target_pulses = new_target * PULSES_PER_MM;
  position_control_mode = true;
  precision.homing_mode = false;
  precision.move_start_time = millis();
  pid_controller.reset();
  
  Serial.print("✓ Moving ");
  Serial.print(distance > 0 ? "+" : "");
  Serial.print(distance, 3);
  Serial.print("mm to position: ");
  Serial.print(new_target, 3);
  Serial.println("mm");
}

void parsePulsesCommand(String params[], int count) {
  if (precision.emergency_stop_active) {
    Serial.println("✗ Emergency stop active! Send RESET first.");
    return;
  }
  
  long target_pulses = params[1].toInt();
  int speed = (count > 2) ? params[2].toInt() : 0;
  
  if (abs(target_pulses) > 50000) {
    Serial.println("✗ Pulse count too large! Max ±50000 pulses");
    return;
  }
  
  float distance_mm = target_pulses * MM_PER_PULSE;
  float new_target = precision.position_mm + distance_mm;
  
  if (new_target < MIN_TRAVEL_MM || new_target > MAX_TRAVEL_MM) {
    Serial.print("✗ Move would exceed travel limits! Target: ");
    Serial.print(new_target, 3);
    Serial.println("mm");
    return;
  }
  
  if (speed > 0) {
    precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  
  precision.target_position_mm = new_target;
  precision.target_pulses = precision.position_pulses + target_pulses;
  position_control_mode = true;
  precision.homing_mode = false;
  precision.move_start_time = millis();
  pid_controller.reset();
  
  Serial.print("✓ Moving ");
  Serial.print(target_pulses > 0 ? "+" : "");
  Serial.print(target_pulses);
  Serial.print(" pulses (");
  
  float revolutions = target_pulses / (float)ENCODER_PPR;
  if (abs(revolutions) >= 1.0) {
    Serial.print(revolutions, 2);
    Serial.print(" rev, ");
  }
  
  Serial.print(distance_mm, 3);
  Serial.print("mm) to position: ");
  Serial.print(new_target, 3);
  Serial.println("mm");
}

void parseJogCommand(String cmd) {
  if (precision.emergency_stop_active) {
    Serial.println("✗ Emergency stop active! Send RESET first.");
    return;
  }
  
  position_control_mode = false;
  precision.homing_mode = false;
  sequence.stop();
  
  if (cmd == "JOG_STOP") {
    stopMotor();
    Serial.println("✓ Jog stopped");
  }
  else if (cmd.startsWith("JOG_FWD")) {
    int speed = precision.jog_speed;
    if (cmd.length() > 7) {
      speed = cmd.substring(8).toInt();
      precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    }
    setMotorSpeed(speed, 1);
    Serial.print("✓ Jogging forward at speed ");
    Serial.println(speed);
  }
  else if (cmd.startsWith("JOG_REV")) {
    int speed = precision.jog_speed;
    if (cmd.length() > 7) {
      speed = cmd.substring(8).toInt();
      precision.jog_speed = constrain(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    }
    setMotorSpeed(speed, -1);
    Serial.print("✓ Jogging reverse at speed ");
    Serial.println(speed);
  }
}

// ========== DISPLAY FUNCTIONS ==========
void displayPrecisionData() {
  if (detailed_mode) {
    displayDetailedData();
  } else {
    displayCompactData();
  }
}

void displayCompactData() {
  Serial.print("Pos: ");
  Serial.print(precision.position_mm, 3);
  Serial.print("mm (");
  Serial.print(precision.position_pulses);
  Serial.print(") | Vel: ");
  Serial.print(precision.velocity_mm_per_sec, 2);
  Serial.print("mm/s | Motor: ");
  Serial.print(precision.motor_speed);
  Serial.print(" ");
  if (precision.motor_direction > 0) Serial.print("FWD");
  else if (precision.motor_direction < 0) Serial.print("REV");
  else Serial.print("STOP");
  
  if (position_control_mode) {
    Serial.print(" | Target: ");
    Serial.print(precision.target_position_mm, 2);
    Serial.print("mm");
  }
  
  if (sequence.is_running) {
    Serial.print(" | SEQ: ");
    Serial.print(sequence.current_step + 1);
    Serial.print("/");
    Serial.print(sequence.total_steps);
    if (sequence.is_paused) Serial.print(" PAUSED");
    if (sequence.in_delay) Serial.print(" DELAY");
  }
  
  Serial.println();
}

void displayDetailedData() {
  Serial.println("=== ADVANCED MOTOR + SEQUENCE STATUS ===");
  Serial.print("Position: ");
  Serial.print(precision.position_mm, 4);
  Serial.print("mm (");
  Serial.print(precision.position_pulses);
  Serial.println(" pulses)");
  
  if (position_control_mode) {
    Serial.print("Target: ");
    Serial.print(precision.target_position_mm, 4);
    Serial.print("mm | Error: ");
    Serial.print(precision.position_error_mm * 1000, 1);
    Serial.println("µm");
  }
  
  Serial.print("Motor: Speed=");
  Serial.print(precision.motor_speed);
  Serial.print(" Dir=");
  if (precision.motor_direction > 0) Serial.print("FWD");
  else if (precision.motor_direction < 0) Serial.print("REV");
  else Serial.print("STOP");
  Serial.print(" | Moving: ");
  Serial.println(precision.is_moving ? "YES" : "NO");
  
  if (sequence.is_running) {
    Serial.print("Sequence: Step ");
    Serial.print(sequence.current_step + 1);
    Serial.print("/");
    Serial.print(sequence.total_steps);
    Serial.print(" (");
    if (sequence.total_steps > 0) {
      Serial.print((sequence.completed_steps * 100) / sequence.total_steps);
    } else {
      Serial.print("0");
    }
    Serial.print("%)");
    if (sequence.is_paused) Serial.print(" [PAUSED]");
    if (sequence.in_delay) Serial.print(" [DELAYING]");
    Serial.println();
  }
  
  Serial.println("========================================");
}

// ========== SYSTEM FUNCTIONS ==========
void resetSystem() {
  stopMotor();
  position_control_mode = false;
  precision.homing_mode = false;
  sequence.stop();
  sequence.reset();
  
  noInterrupts();
  encoder_count = 0;
  interrupts();
  
  memset(&precision, 0, sizeof(precision));
  precision.start_time = millis();
  precision.last_update_time = millis();
  precision.emergency_stop_active = false;
  precision.limits_enabled = true;
  precision.jog_speed = DEFAULT_SPEED;
  
  pid_controller.reset();
  
  Serial.println("✓ System reset complete - All sequences cleared");
}

void zeroPosition() {
  float old_position = precision.position_mm;
  
  noInterrupts();
  encoder_count = 0;
  interrupts();
  
  Serial.print("✓ Position zeroed. Previous: ");
  Serial.print(old_position, 3);
  Serial.println("mm");
}

void showCurrentStatus() {
  Serial.println("=== CURRENT STATUS ===");
  Serial.print("Position: ");
  Serial.print(precision.position_mm, 4);
  Serial.print("mm (");
  Serial.print(precision.position_pulses);
  Serial.println(" pulses)");
  
  if (position_control_mode) {
    Serial.print("Target: ");
    Serial.print(precision.target_position_mm, 4);
    Serial.print("mm | Error: ");
    Serial.print(precision.position_error_mm * 1000, 1);
    Serial.println("µm");
  } else if (precision.homing_mode) {
    Serial.println("Mode: HOMING");
  } else if (sequence.is_running) {
    Serial.print("Mode: SEQUENCE (Step ");
    Serial.print(sequence.current_step + 1);
    Serial.print("/");
    Serial.print(sequence.total_steps);
    Serial.println(")");
  } else {
    Serial.println("Mode: Manual/Free positioning");
  }
  
  if (sequence.total_steps > 0) {
    Serial.print("Sequence: ");
    Serial.print(sequence.completed_steps);
    Serial.print("/");
    Serial.print(sequence.total_steps);
    Serial.print(" steps (");
    Serial.print(sequence.is_running ? "RUNNING" : "STOPPED");
    Serial.println(")");
  }
  
  Serial.println("======================");
}

void showSystemInfo() {
  Serial.println("=== SYSTEM SPECIFICATIONS ===");
  Serial.println("Hardware:");
  Serial.print("  Encoder: ");
  Serial.print(ENCODER_PPR);
  Serial.print(" PPR, Pins A=");
  Serial.print(ENCODER_A);
  Serial.print(" B=");
  Serial.println(ENCODER_B);
  
  Serial.println("Performance:");
  Serial.print("  Resolution: ");
  Serial.print(THEORETICAL_ACCURACY, 1);
  Serial.println("µm/pulse");
  Serial.print("  Max Sequence Steps: ");
  Serial.println(MAX_SEQUENCE_STEPS);
  Serial.println("==============================");
}

void printWelcomeMessage() {
  Serial.println("===============================================");
  Serial.println("  ADVANCED DC MOTOR SEQUENCE SYSTEM V4.0 FIXED");
  Serial.println("===============================================");
  Serial.print("Encoder: ");
  Serial.print(ENCODER_PPR);
  Serial.println(" PPR");
  Serial.print("Resolution: ");
  Serial.print(PULSES_PER_MM, 1);
  Serial.print(" pulses/mm (");
  Serial.print(THEORETICAL_ACCURACY, 1);
  Serial.println("µm/pulse)");
  Serial.println();
  Serial.println("Quick Examples:");
  Serial.println("  RUN MOVE 10 150 3000  - Move 10mm, speed 150, delay 3s");
  Serial.println("  COMPLEX MOVE 5 100 1000 GOTO 0 80 2000");
  Serial.println("  SEQ_ADD MOVE 10 120 - Add step to sequence");
  Serial.println("===============================================");
  Serial.println();
}

void printAdvancedHelp() {
  Serial.println("=== ADVANCED MOTOR CONTROL COMMANDS ===");
  Serial.println();
  
  Serial.println("🚀 ADVANCED COMMANDS:");
  Serial.println("Single Complex Commands:");
  Serial.println("  RUN <action> <value> [speed] [delay_ms]");
  Serial.println("    RUN MOVE 10 150 3000   - Move 10mm, speed 150, delay 3s");
  Serial.println("    RUN GOTO 25 120 2000   - Go to 25mm, speed 120, delay 2s");
  Serial.println("    RUN PULSES 500 100 1500 - 1 rev, speed 100, delay 1.5s");
  Serial.println();
  
  Serial.println("Multi-Step Sequences:");
  Serial.println("  COMPLEX <cmd1> <val1> <spd1> <dly1> <cmd2> <val2> <spd2> <dly2>...");
  Serial.println("    COMPLEX MOVE 10 150 2000 GOTO 0 100 3000 MOVE -5 80 1000");
  Serial.println();
  
  Serial.println("Sequence Builder:");
  Serial.println("  SEQ_ADD <action> <value> [speed] [delay_ms] - Add step");
  Serial.println("  SEQ_START           - Start sequence");
  Serial.println("  SEQ_PAUSE/RESUME    - Pause/resume sequence");
  Serial.println("  SEQ_STOP            - Stop sequence");
  Serial.println("  SEQ_SHOW            - Show current sequence");
  Serial.println("  SEQ_CLEAR           - Clear sequence");
  Serial.println();
  
  Serial.println("Basic Commands:");
  Serial.println("  GOTO <mm> [speed]   - Move to absolute position");
  Serial.println("  MOVE <mm> [speed]   - Move relative distance");
  Serial.println("  PULSES <count> [speed] - Move by pulse count");
  Serial.println("  JOG_FWD/REV [speed] - Manual jog");
  Serial.println("  MOTOR_STOP          - Emergency stop");
  Serial.println();
  
  Serial.println("System:");
  Serial.println("  RESET/ZERO/STATUS/INFO - System functions");
  Serial.println("  LIMITS_ON/OFF       - Enable/disable limits");
  Serial.println("  BACKLASH_ON/OFF     - Backlash compensation");
  Serial.println("==========================================");
}

// Utility functions
void startContinuousDisplay() {
  continuous_display = true;
  Serial.println("✓ Continuous display started");
}

void stopContinuousDisplay() {
  continuous_display = false;
  Serial.println("✓ Continuous display stopped");
}

void toggleDetailedMode() {
  detailed_mode = !detailed_mode;
  Serial.print("✓ Display mode: ");
  Serial.println(detailed_mode ? "Detailed" : "Compact");
}
