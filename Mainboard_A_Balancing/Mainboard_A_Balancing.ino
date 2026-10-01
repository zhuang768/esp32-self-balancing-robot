/*
  ESP32 雙輪自平衡機器人 - 主板 A

  功能：MPU6050 校正與互補濾波、PID 平衡、TB6612FNG 雙馬達、
  跌倒與感測器異常保護、OLED、可選蜂鳴器與序列控制。

  重要：此程式可編譯，但馬達方向、IMU 軸向與 PID 仍須實機驗證。
  開機後馬達預設停止，必須從序列監控輸入 e 才會啟用平衡。
*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Wire.h>
#include <math.h>

// 接線或安裝方向改變時，只修改這個設定區。
namespace Config {
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t PWMA = 16;
constexpr uint8_t AIN1 = 17;
constexpr uint8_t AIN2 = 18;
constexpr uint8_t PWMB = 19;
constexpr uint8_t BIN1 = 23;
constexpr uint8_t BIN2 = 25;
constexpr uint8_t BUZZER = 27;

constexpr bool MOTOR_A_INVERTED = false;
constexpr bool MOTOR_B_INVERTED = false;
constexpr float IMU_DIRECTION = 1.0F;

constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

// 蜂鳴器類型尚未確認，所以預設關閉。
constexpr bool BUZZER_ENABLED = false;
constexpr bool BUZZER_ACTIVE_HIGH = true;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_PERIOD_US = 5000;
constexpr uint32_t DISPLAY_PERIOD_MS = 100;
constexpr int CALIBRATION_SAMPLES = 600;

constexpr float ENABLE_ANGLE_DEG = 10.0F;
constexpr float FALL_ANGLE_DEG = 35.0F;
constexpr int MOTOR_DEADZONE = 40;
constexpr int MAX_PWM = 120;

constexpr double KP = 25.0;
constexpr double KI = 0.8;
constexpr double KD = 1.2;
}  // namespace Config

enum class RobotState : uint8_t { Booting, Ready, Balancing, Fallen, Fault };

MPU6050 mpu;
Adafruit_SSD1306 display(
    Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, -1);

double pidInput = 0.0;
double pidOutput = 0.0;
double pidSetpoint = 0.0;
double kp = Config::KP;
double ki = Config::KI;
double kd = Config::KD;
PID balancePid(&pidInput, &pidOutput, &pidSetpoint, kp, ki, kd, DIRECT);

RobotState state = RobotState::Booting;
bool displayAvailable = false;
float angleDeg = 0.0F;
float angleZeroDeg = 0.0F;
float gyroYOffset = 0.0F;
uint32_t lastControlUs = 0;
uint32_t lastDisplayMs = 0;

const char* stateLabel(RobotState value) {
  switch (value) {
    case RobotState::Booting: return "BOOT";
    case RobotState::Ready: return "READY";
    case RobotState::Balancing: return "BALANCE";
    case RobotState::Fallen: return "FALLEN";
    case RobotState::Fault: return "FAULT";
  }
  return "UNKNOWN";
}

void setBuzzer(bool on) {
  if (!Config::BUZZER_ENABLED) {
    digitalWrite(Config::BUZZER, Config::BUZZER_ACTIVE_HIGH ? LOW : HIGH);
    return;
  }
  const bool level = Config::BUZZER_ACTIVE_HIGH ? on : !on;
  digitalWrite(Config::BUZZER, level ? HIGH : LOW);
}

void stopMotors() {
  analogWrite(Config::PWMA, 0);
  analogWrite(Config::PWMB, 0);
  digitalWrite(Config::AIN1, LOW);
  digitalWrite(Config::AIN2, LOW);
  digitalWrite(Config::BIN1, LOW);
  digitalWrite(Config::BIN2, LOW);
}

void setOneMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int command,
                 bool inverted) {
  command = constrain(command, -Config::MAX_PWM, Config::MAX_PWM);
  if (inverted) command = -command;

  if (command == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  const bool forward = command > 0;
  const int magnitude = abs(command);
  const int pwm = map(magnitude, 1, Config::MAX_PWM,
                      Config::MOTOR_DEADZONE, Config::MAX_PWM);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void driveBothMotors(int command) {
  setOneMotor(Config::PWMA, Config::AIN1, Config::AIN2, command,
              Config::MOTOR_A_INVERTED);
  setOneMotor(Config::PWMB, Config::BIN1, Config::BIN2, command,
              Config::MOTOR_B_INVERTED);
}

void enterSafeState(RobotState nextState) {
  stopMotors();
  balancePid.SetMode(MANUAL);
  pidOutput = 0.0;
  state = nextState;
  setBuzzer(nextState == RobotState::Fallen || nextState == RobotState::Fault);
}

bool readImu(float dtSeconds) {
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  const float accAngle = Config::IMU_DIRECTION *
      (atan2f(static_cast<float>(ax), static_cast<float>(az)) * 180.0F / PI -
       angleZeroDeg);
  const float gyroRate = Config::IMU_DIRECTION *
      (static_cast<float>(gy) - gyroYOffset) / 131.0F;

  if (!isfinite(accAngle) || !isfinite(gyroRate)) return false;
  angleDeg = 0.98F * (angleDeg + gyroRate * dtSeconds) + 0.02F * accAngle;
  return isfinite(angleDeg);
}

bool calibrateImu() {
  enterSafeState(RobotState::Booting);
  Serial.println("校正中：請讓機器人保持靜止並接近直立...");

  double angleSum = 0.0;
  int64_t gyroSum = 0;
  for (int i = 0; i < Config::CALIBRATION_SAMPLES; ++i) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angleSum += atan2f(static_cast<float>(ax), static_cast<float>(az)) *
                180.0F / PI;
    gyroSum += gy;
    delay(5);
  }

  angleZeroDeg = static_cast<float>(angleSum / Config::CALIBRATION_SAMPLES);
  gyroYOffset = static_cast<float>(gyroSum) / Config::CALIBRATION_SAMPLES;
  angleDeg = 0.0F;
  pidInput = 0.0;
  state = RobotState::Ready;
  setBuzzer(false);
  Serial.println("校正完成。輸入 e 啟用平衡，x 停止，c 重新校正。");
  return true;
}

void enableBalancing() {
  if (state == RobotState::Fault) {
    Serial.println("感測器故障狀態，不能啟用。");
    return;
  }
  if (fabsf(angleDeg) > Config::ENABLE_ANGLE_DEG) {
    Serial.println("角度離直立位置太遠，拒絕啟用。");
    return;
  }

  stopMotors();
  pidInput = angleDeg;
  pidOutput = 0.0;
  balancePid.SetMode(MANUAL);
  balancePid.SetMode(AUTOMATIC);
  state = RobotState::Balancing;
  setBuzzer(false);
  Serial.println("平衡控制已啟用。輸入 x 可立即停止。");
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'e' || command == 'E') {
      enableBalancing();
    } else if (command == 'x' || command == 'X') {
      enterSafeState(RobotState::Ready);
      Serial.println("馬達已停止。");
    } else if (command == 'c' || command == 'C') {
      calibrateImu();
    } else if (command == 'p' || command == 'P') {
      Serial.printf("state=%s angle=%.2f output=%.1f\n", stateLabel(state),
                    angleDeg, pidOutput);
    } else if (command == 'h' || command == 'H') {
      Serial.println("指令：e 啟用、x 停止、c 校正、p 狀態、h 說明");
    }
  }
}

void updateDisplay() {
  if (!displayAvailable ||
      millis() - lastDisplayMs < Config::DISPLAY_PERIOD_MS) return;
  lastDisplayMs = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Self-Balance Robot");
  display.print("State: "); display.println(stateLabel(state));
  display.print("Angle: "); display.println(angleDeg, 1);
  display.print("PID: "); display.println(pidOutput, 0);
  display.print("Limit: "); display.println(Config::MAX_PWM);
  display.display();
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);

  pinMode(Config::PWMA, OUTPUT);
  pinMode(Config::AIN1, OUTPUT);
  pinMode(Config::AIN2, OUTPUT);
  pinMode(Config::PWMB, OUTPUT);
  pinMode(Config::BIN1, OUTPUT);
  pinMode(Config::BIN2, OUTPUT);
  pinMode(Config::BUZZER, OUTPUT);
  stopMotors();
  setBuzzer(false);

  Wire.begin(Config::I2C_SDA, Config::I2C_SCL);
  Wire.setClock(400000);

  displayAvailable = display.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDRESS);
  if (displayAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting safely...");
    display.display();
  } else {
    Serial.println("OLED 未連線；控制器會繼續執行。");
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    enterSafeState(RobotState::Fault);
    Serial.println("MPU6050 連線失敗，馬達保持停止。");
    return;
  }

  balancePid.SetOutputLimits(-Config::MAX_PWM, Config::MAX_PWM);
  balancePid.SetSampleTime(Config::CONTROL_PERIOD_US / 1000);
  balancePid.SetMode(MANUAL);
  calibrateImu();
  lastControlUs = micros();
}

void loop() {
  handleSerial();
  updateDisplay();

  if (state == RobotState::Fault) {
    stopMotors();
    return;
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  if (elapsedUs < Config::CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;

  const float dtSeconds = elapsedUs / 1000000.0F;
  if (!readImu(dtSeconds)) {
    enterSafeState(RobotState::Fault);
    Serial.println("IMU 讀值異常，馬達已停止。");
    return;
  }

  pidInput = angleDeg;
  if (fabsf(angleDeg) > Config::FALL_ANGLE_DEG) {
    if (state != RobotState::Fallen) {
      enterSafeState(RobotState::Fallen);
      Serial.println("偵測到跌倒，馬達已停止；扶正後輸入 c 重新校正。");
    }
    return;
  }

  if (state != RobotState::Balancing) {
    stopMotors();
    return;
  }

  if (balancePid.Compute()) {
    driveBothMotors(static_cast<int>(pidOutput));
  }
}
