#include <Wire.h>
#include <math.h>

#define MPU_ADDR           0x68
#define SAMPLE_RATE_HZ     100      
#define PRINT_INTERVAL_MS  100      

#define KF_Q_ANGLE         0.001f
#define KF_Q_BIAS          0.003f
#define KF_R_MEASURE       0.03f

struct KalmanFilter {
  float angle = 0; float bias = 0; float P[2][2] = {{0,0},{0,0}}; [cite: 7]
  float update(float newAngle, float newRate, float dt) {
    float rate = newRate - bias; angle += dt * rate; [cite: 8, 9]
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + KF_Q_ANGLE); [cite: 9]
    P[0][1] -= dt * P[1][1]; P[1][0] -= dt * P[1][1]; P[1][1] += KF_Q_BIAS * dt; [cite: 10]
    float S = P[0][0] + KF_R_MEASURE; float K0 = P[0][0] / S; float K1 = P[1][0] / S; [cite: 11]
    float y = newAngle - angle; angle += K0 * y; bias += K1 * y; [cite: 12]
    float P00_tmp = P[0][0]; float P01_tmp = P[0][1]; [cite: 13]
    P[0][0] -= K0 * P00_tmp; P[0][1] -= K0 * P01_tmp; [cite: 13]
    P[1][0] -= K1 * P00_tmp; P[1][1] -= K1 * P01_tmp; [cite: 14]
    return angle;
  }
};

float ax_off = 0.0f, ay_off = 0.0f, az_off = 0.0f; [cite: 3]
float gx_off = 0.0f, gy_off = 0.0f, gz_off = 0.0f; [cite: 4]
int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz; [cite: 4]
unsigned long prev_us = 0; [cite: 5]
unsigned long last_print_ms = 0; [cite: 5]

float yawEstimate = 0.0f; [cite: 6]
KalmanFilter kfRoll; KalmanFilter kfPitch; [cite: 14]

void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false); [cite: 15]
  Wire.requestFrom(MPU_ADDR, 14, true); [cite: 15]
  if (Wire.available() >= 14) {
    raw_ax = Wire.read()<<8 | Wire.read(); raw_ay = Wire.read()<<8 | Wire.read(); [cite: 16]
    raw_az = Wire.read()<<8 | Wire.read(); [cite: 17]
    Wire.read(); Wire.read();
    raw_gx = Wire.read()<<8 | Wire.read(); raw_gy = Wire.read()<<8 | Wire.read(); [cite: 17]
    raw_gz = Wire.read()<<8 | Wire.read(); [cite: 18]
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000);
  
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission(); delay(50); [cite: 18]
  
  readMPU();
  float initAx = (raw_ax / 16384.0f) + ax_off; float initAy = (raw_ay / 16384.0f) + ay_off; float initAz = (raw_az / 16384.0f) + az_off; [cite: 51, 52, 53]
  float startRoll = atan2f(initAy, initAz) * 180.0f / M_PI; [cite: 53]
  float startPitch = atan2f(-initAx, sqrtf(initAy*initAy + initAz*initAz)) * 180.0f / M_PI; [cite: 53, 54]
  
  kfRoll.angle = startRoll; kfPitch.angle = startPitch; [cite: 54]
  prev_us = micros(); [cite: 55]
}

void loop() {
  readMPU();
  unsigned long now_us = micros(); [cite: 57]
  float dt = (float)(now_us - prev_us) / 1e6f; [cite: 57]
  if(dt <= 0 || dt > 0.5f) dt = 1.0f / SAMPLE_RATE_HZ; [cite: 58]
  prev_us = now_us; [cite: 58]

  float Ax_cal = (raw_ax / 16384.0f) + ax_off; [cite: 59]
  float Ay_cal = (raw_ay / 16384.0f) + ay_off; [cite: 59, 60]
  float Az_cal = (raw_az / 16384.0f) + az_off; [cite: 60]
  float Gx_cal = (raw_gx / 131.0f) - gx_off; [cite: 60]
  float Gy_cal = (raw_gy / 131.0f) - gy_off; [cite: 61]
  float Gz_cal = (raw_gz / 131.0f) - gz_off; [cite: 61]

  float accRoll  = atan2f(Ay_cal, Az_cal) * 180.0f / M_PI; [cite: 108]
  float accPitch = atan2f(-Ax_cal, sqrtf(Ay_cal*Ay_cal + Az_cal*Az_cal)) * 180.0f / M_PI; [cite: 109]
  
  float kalRoll  = kfRoll.update(accRoll, Gx_cal, dt); [cite: 109]
  float kalPitch = kfPitch.update(accPitch, Gy_cal, dt); [cite: 110]
  yawEstimate   += Gz_cal * dt; [cite: 110]
  
  if (millis() - last_print_ms >= PRINT_INTERVAL_MS) {
    last_print_ms = millis(); [cite: 111]
    Serial.print(F("Roll: "));   Serial.print(kalRoll, 1);    Serial.print(F("o")); [cite: 111]
    Serial.print(F(" | Pitch: ")); Serial.print(kalPitch, 1);  Serial.print(F("o")); [cite: 112]
    Serial.print(F(" | Yaw: "));   Serial.print(yawEstimate, 1); Serial.println(F("o")); [cite: 112]
  }
  while (micros() - now_us < (1000000L / SAMPLE_RATE_HZ)); [cite: 113]
}