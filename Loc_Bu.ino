#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

// HỆ SỐ LỌC BÙ
#define ALPHA 0.98

// OFFSET
const float OFFSET_AX = 0.035075;
const float OFFSET_AY = 0.024395;
const float OFFSET_AZ = 0.029585;
const float OFFSET_GY = 0.49782;

float comp_angle = 0.0;
unsigned long last_time = 0;

struct IMUData {
  float ax;
  float ay;
  float az;
  float gy;
};

void mpuInit() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
}

IMUData readIMU() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  Wire.read();
  Wire.read();

  int16_t raw_gy = (Wire.read() << 8) | Wire.read();

  IMUData d;

  d.ax = (raw_ax / 16384.0) + OFFSET_AX;
  d.ay = (raw_ay / 16384.0) + OFFSET_AY;
  d.az = (raw_az / 16384.0) + OFFSET_AZ;

  d.gy = (raw_gy / 131.0) - OFFSET_GY;

  return d;
}

void setup() {

  Serial.begin(115200);

  Wire.begin();

  mpuInit();

  delay(500);

  IMUData data = readIMU();

  comp_angle =
      atan2(-data.ax,
            sqrt(data.ay * data.ay +
                 data.az * data.az))
      * 180.0 / PI;

  last_time = micros();

  Serial.println("Complementary Filter MPU6050");
  Serial.print("Alpha = ");
  Serial.println(ALPHA, 2);
}

void loop() {

  unsigned long now = micros();

  float dt =
      (float)(now - last_time)
      / 1000000.0;

  if (dt <= 0 || dt > 0.5)
    dt = 0.01;

  last_time = now;

  IMUData data = readIMU();

  float raw_accel_angle =
      atan2(-data.ax,
            sqrt(data.ay * data.ay +
                 data.az * data.az))
      * 180.0 / PI;

  comp_angle =
      ALPHA *
      (comp_angle + data.gy * dt)
      +
      (1.0 - ALPHA)
      * raw_accel_angle;

  Serial.print("Raw_Accel:");
  Serial.print(raw_accel_angle, 2);

  Serial.print(",");

  Serial.print("Complementary_Filter:");
  Serial.println(comp_angle, 2);

  delay(10);
}