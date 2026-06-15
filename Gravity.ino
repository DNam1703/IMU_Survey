#include <Wire.h>
#include <math.h>

#define MPU_ADDR           0x68

float ax_off = -0.02f, ay_off = 0.01f, az_off = -0.05f; // Điền offset từ chế độ thống kê vào đây
int16_t raw_ax, raw_ay, raw_az;
bool prompt_angle = true; [cite: 5]

void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false); [cite: 15]
  Wire.requestFrom(MPU_ADDR, 14, true); [cite: 15]
  if (Wire.available() >= 14) {
    raw_ax = Wire.read()<<8 | Wire.read(); raw_ay = Wire.read()<<8 | Wire.read(); [cite: 16]
    raw_az = Wire.read()<<8 | Wire.read(); [cite: 17]
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000); 
  
  // Khởi động MPU
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission(); delay(50); [cite: 18]
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); [cite: 19]
}

void loop() {
  if (prompt_angle) {
    while(Serial.available() > 0) { Serial.read(); } [cite: 82, 83]
    Serial.println(F("\n>> [YEU CAU]: Chinh den goc do tiep theo (0° -> 90° quanh truc Y).")); [cite: 84]
    Serial.println(F(">> Nhap gia tri goc vao o Chat va an ENTER...")); [cite: 85]
    prompt_angle = false; [cite: 85]
  }

  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n'); [cite: 86]
    inputStr.trim(); [cite: 86]
    if (inputStr.length() > 0) {
      int current_angle = inputStr.toInt(); [cite: 87]

      double sum_ax = 0, sum_ay = 0, sum_az = 0; [cite: 88]
      for (int k = 0; k < 50; k++) { [cite: 89]
        readMPU();
        sum_ax += (raw_ax / 16384.0f) + ax_off; [cite: 90]
        sum_ay += (raw_ay / 16384.0f) + ay_off; [cite: 90, 91]
        sum_az += (raw_az / 16384.0f) + az_off; [cite: 91]
        delay(5);
      }
      float avg_ax = sum_ax / 50.0f; float avg_ay = sum_ay / 50.0f; float avg_az = sum_az / 50.0f; [cite: 91, 92]
      float magnitude_a = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az); [cite: 92]

      Serial.println(F("==================== KET QUA KHAO SAT ====================")); [cite: 93]
      Serial.print(F("Ax: ")); Serial.print(avg_ax, 3); Serial.print(F(" g | Ay: ")); Serial.print(avg_ay, 3); Serial.print(F(" g | Az: ")); Serial.print(avg_az, 3); Serial.println(F(" g")); [cite: 94, 95]
      Serial.print(F("Tong vector |a|:   ")); Serial.print(magnitude_a, 3); Serial.println(F(" g")); [cite: 95, 96]
      Serial.print(F("Phan bo vector gia toc tai goc ")); Serial.print(current_angle); Serial.println(F("°")); [cite: 96]
      Serial.println(F("=========================================================="));
      
      prompt_angle = true; [cite: 97]
    }
  }
}