// ================== CẤU HÌNH PHẦN CỨNG ==================
// BTS7960 Left Motor
#define LEFT_MOTOR_RPWM 5
#define LEFT_MOTOR_LPWM 6
#define LEFT_MOTOR_R_EN 7
#define LEFT_MOTOR_L_EN 8

// BTS7960 Right Motor
#define RIGHT_MOTOR_RPWM 9
#define RIGHT_MOTOR_LPWM 10
#define RIGHT_MOTOR_R_EN 11
#define RIGHT_MOTOR_L_EN 12

// Encoder
#define ENCODER_LEFT_A  18
#define ENCODER_LEFT_B  20
#define ENCODER_RIGHT_A 19
#define ENCODER_RIGHT_B 21

// 5 mắt dò line
#define SENSOR_1 22  // Trái ngoài
#define SENSOR_2 23  // Trái giữa
#define SENSOR_3 24  // Trung tâm
#define SENSOR_4 25  // Phải giữa
#define SENSOR_5 26  // Phải ngoài

// ================== BIẾN TOÀN CỤC ==================
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

int sensor[5];
int lastLineError = 0;
bool lostLine = false;
unsigned long lastSeenLine = 0;

// PID tốc độ
float Kp_speed = 1.5, Ki_speed = 0.1, Kd_speed = 0.5;
float lastErrorSpeedLeft = 0, lastErrorSpeedRight = 0;
float integralLeft = 0, integralRight = 0;

// PID line
float Kp_line = 30, Ki_line = 0, Kd_line = 20;
float integralLine = 0;

// Tốc độ đặt
int baseSpeed = 180;   // chỉnh tùy xe

// Map điều hướng
String route[] = {"LEFT", "STRAIGHT", "RIGHT", "STRAIGHT", "LEFT"}; 
int routeLength = 5;
int currentStep = 0;

// ================== HÀM NGẮT ENCODER ==================
void encoderLeftA() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B))
    encoderCountLeft++;
  else
    encoderCountLeft--;
}

void encoderRightA() {
  if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B))
    encoderCountRight++;
  else
    encoderCountRight--;
}

// ================== HÀM ĐIỀU KHIỂN MOTOR ==================
void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    analogWrite(LEFT_MOTOR_RPWM, pwm);
    analogWrite(LEFT_MOTOR_LPWM, 0);
  } else {
    analogWrite(LEFT_MOTOR_RPWM, 0);
    analogWrite(LEFT_MOTOR_LPWM, -pwm);
  }
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    analogWrite(RIGHT_MOTOR_RPWM, pwm);
    analogWrite(RIGHT_MOTOR_LPWM, 0);
  } else {
    analogWrite(RIGHT_MOTOR_RPWM, 0);
    analogWrite(RIGHT_MOTOR_LPWM, -pwm);
  }
}

// ================== HÀM ĐỌC CẢM BIẾN LINE ==================
void readSensors() {
  sensor[0] = digitalRead(SENSOR_1);
  sensor[1] = digitalRead(SENSOR_2);
  sensor[2] = digitalRead(SENSOR_3);
  sensor[3] = digitalRead(SENSOR_4);
  sensor[4] = digitalRead(SENSOR_5);
}

int getLineError() {
  int weights[5] = {-2, -1, 0, 1, 2};
  long sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (sensor[i] == 0) { // line đen
      sum += weights[i];
      count++;
    }
  }

  if (count == 0) {
    lostLine = true;
    return lastLineError; // giữ hướng cũ
  } else {
    lostLine = false;
    lastLineError = sum / count;
    lastSeenLine = millis();
    return lastLineError;
  }
}

// ================== PID TỐC ĐỘ ==================
int PID_speed(int target, long count, float &lastError, float &integral) {
  float error = target - count;
  integral += error;
  float derivative = error - lastError;
  lastError = error;
  return (int)(Kp_speed * error + Ki_speed * integral + Kd_speed * derivative);
}

// ================== PID LINE ==================
int PID_line(int error) {
  integralLine += error;
  float derivative = error - lastLineError;
  return (int)(Kp_line * error + Ki_line * integralLine + Kd_line * derivative);
}

// ================== PHÁT HIỆN NGÃ RẼ ==================
String detectJunction() {
  if (sensor[0] == 0 && sensor[4] == 0) return "CROSS"; // Ngã tư
  if (sensor[0] == 0) return "LEFT";   // Ngã ba trái
  if (sensor[4] == 0) return "RIGHT";  // Ngã ba phải
  return "NONE";
}

// ================== XỬ LÝ NGÃ RẼ ==================
void handleJunction(String junction) {
  if (currentStep >= routeLength) return; // hết map

  String action = route[currentStep];

  if (junction == "LEFT" || junction == "RIGHT" || junction == "CROSS") {
    // Dừng ngắn trước khi rẽ
    setMotorLeft(0); setMotorRight(0);
    delay(100);

    if (action == "LEFT") {
      setMotorLeft(-120); setMotorRight(120);
      delay(300); // thời gian xoay
    } else if (action == "RIGHT") {
      setMotorLeft(120); setMotorRight(-120);
      delay(300);
    } else if (action == "STRAIGHT") {
      setMotorLeft(baseSpeed); setMotorRight(baseSpeed);
      delay(300);
    }

    currentStep++; // sang bước tiếp theo
  }
}

// ================== SETUP ==================
void setup() {
  pinMode(LEFT_MOTOR_RPWM, OUTPUT);
  pinMode(LEFT_MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightA, CHANGE);

  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);

  Serial.begin(9600);
}

// ================== LOOP ==================
void loop() {
  // Reset encoder count mỗi vòng lặp
  encoderCountLeft = 0;
  encoderCountRight = 0;
  delay(50);

  // PID tốc độ
  int pidLeft = PID_speed(baseSpeed, encoderCountLeft, lastErrorSpeedLeft, integralLeft);
  int pidRight = PID_speed(baseSpeed, encoderCountRight, lastErrorSpeedRight, integralRight);

  // PID line
  readSensors();
  int lineError = getLineError();
  int pidLine = PID_line(lineError);

  int leftPWM, rightPWM;

  String junction = detectJunction();

  if (junction != "NONE") {
    handleJunction(junction);
  } else {
    if (!lostLine) {
      // Đi theo PID line
      leftPWM = baseSpeed + pidLeft - pidLine;
      rightPWM = baseSpeed + pidRight + pidLine;
    } else {
      if (millis() - lastSeenLine < 200) {
        // Có thể cua gắt
        if (lastLineError < 0) {
          leftPWM  = baseSpeed - 80;
          rightPWM = baseSpeed + 80;
        } else {
          leftPWM  = baseSpeed + 80;
          rightPWM = baseSpeed - 80;
        }
      } else {
        // Mất line thật sự -> xoay tại chỗ
        if (lastLineError < 0) {
          leftPWM = -120;
          rightPWM = 120;
        } else {
          leftPWM = 120;
          rightPWM = -120;
        }
      }
    }

    // Giới hạn PWM
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    setMotorLeft(leftPWM);
    setMotorRight(rightPWM);
  }

  // Debug
  Serial.print("Step: "); Serial.print(currentStep);
  Serial.print("  Junction: "); Serial.print(junction);
  Serial.print("  LineErr: "); Serial.print(lineError);
  Serial.print("  Lost: "); Serial.print(lostLine);
  Serial.print("  L_PWM: "); Serial.print(leftPWM);
  Serial.print("  R_PWM: "); Serial.println(rightPWM);
}
