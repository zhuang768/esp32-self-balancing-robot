/*
  ============================================================
  自平衡機器人 - 馬達驅動硬體獨立測試 (TB6612FNG + TT 馬達)
  ============================================================
  此程式不依賴 MPU6050 與 OLED，僅用於驗證馬達供電與轉向。
*/

#define PWMA 16
#define AIN1 17
#define AIN2 18
#define PWMB 19
#define BIN1 23
#define BIN2 25

const int TEST_SPEED = 80;
const unsigned long TEST_DURATION_MS = 500;

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // 預設保持停止，連接 USB 或重新啟動時不會自行轉動。
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  Serial.println("================================");
  Serial.println("安全馬達測試已就緒，預設停止");
  Serial.println("輸入 1：A 正轉；2：B 正轉；3：雙輪正轉；4：雙輪反轉；s：停止");
  Serial.println("================================");
}

void setMotorA(int speed) {
  // speed: -255 ~ 255
  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void setMotorB(int speed) {
  // speed: -255 ~ 255
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  const char command = Serial.read();
  setMotorA(0);
  setMotorB(0);

  if (command == '1') {
    Serial.println("馬達 A 低速正轉 0.5 秒");
    setMotorA(TEST_SPEED);
  } else if (command == '2') {
    Serial.println("馬達 B 低速正轉 0.5 秒");
    setMotorB(TEST_SPEED);
  } else if (command == '3') {
    Serial.println("雙輪低速正轉 0.5 秒");
    setMotorA(TEST_SPEED);
    setMotorB(TEST_SPEED);
  } else if (command == '4') {
    Serial.println("雙輪低速反轉 0.5 秒");
    setMotorA(-TEST_SPEED);
    setMotorB(-TEST_SPEED);
  } else if (command == 's' || command == 'S') {
    Serial.println("已停止");
    return;
  } else if (command == '\n' || command == '\r') {
    return;
  } else {
    Serial.println("未知指令；可輸入 1、2、3、4 或 s");
    return;
  }

  delay(TEST_DURATION_MS);
  setMotorA(0);
  setMotorB(0);
  Serial.println("測試完成，已自動停止");
}
