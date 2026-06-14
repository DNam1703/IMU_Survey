#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

unsigned long last_time = 0;
// Tăng chu kỳ lên 20000 micro giây (20ms) = Tần số 50Hz (Chậm hơn 10 lần bản cũ)
const unsigned long SAMPLE_PERIOD_US = 20000; 

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); 
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  mpuInit();
  delay(200);
  last_time = micros();
}

void loop() {
  // Chờ cho đến khi đủ 20ms
  while (micros() - last_time < SAMPLE_PERIOD_US);
  last_time += SAMPLE_PERIOD_US;

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 6, (int)true); 
  
  float ax = ((int16_t)((Wire.read() << 8) | Wire.read()) / 16384.0);
  float ay = ((int16_t)((Wire.read() << 8) | Wire.read()) / 16384.0);
  float az = ((int16_t)((Wire.read() << 8) | Wire.read()) / 16384.0);
  
  // Tính độ lớn gia tốc tổng hợp |a|
  float accel_mag = sqrt(ax * ax + ay * ay + az * az);
  
  // Tính toán biên độ xung (Trừ đi 1g tĩnh)
  float shock_val = abs(accel_mag - 1.0);
  
  // Định dạng đầu ra cho Serial Plotter
  Serial.print("Shock_Amplitude:");
  Serial.println(shock_val, 4);
}