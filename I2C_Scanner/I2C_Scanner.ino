#include <Wire.h>

void setup() {
  Serial.begin(115200);
  // 強制指定 SDA 為 32，SCL 為 33 (避開可能損壞的 21/22)
  Wire.begin(32, 33);
  Serial.println("\n==================================");
  Serial.println("I2C 裝置掃描器 (I2C Scanner) 啟動");
  Serial.println("==================================");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("開始掃描 I2C 匯流排...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("=> 成功！找到 I2C 裝置，位址 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("=> 警告：位址 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      Serial.println(" 發生未知錯誤");
    }
  }

  if (nDevices == 0) {
    Serial.println("=> 找不到任何裝置！請檢查接線。");
  } else {
    Serial.println("=> 掃描完畢。");
  }
  Serial.println("----------------------------------\n");

  delay(5000); // 每 5 秒重新掃描一次
}
