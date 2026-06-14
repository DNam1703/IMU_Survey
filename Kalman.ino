#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

//================= OFFSET =================
const float OFFSET_AX = 0.035075;
const float OFFSET_AY = 0.024395;
const float OFFSET_AZ = 0.029585;
const float OFFSET_GY = 0.49782;

//============ COMPLEMENTARY FILTER ============
#define ALPHA 0.95
float comp_angle = 0.0;

//=============== KALMAN FILTER ===============
const float Q_angle   = 0.082778;
const float Q_bias    = 0.008863;
const float R_measure = 0.029046;

float kalman_angle = 0.0;
float kalman_bias  = 0.0;

float P[2][2] = {
  {0.0, 0.0},
  {0.0, 0.0}
};

unsigned long last_time;

// Thêm biến quản lý chu kỳ chạy cố định (Định dạng khoảng rộng đồ thị)
unsigned long timer_plotter = 0;
const unsigned long SAMPLE_PERIOD_MS = 10; // Lấy mẫu chuẩn mỗi 10ms (100Hz)

//=============================================
struct IMUData {
  float ax;
  float ay;
  float az;
  float gy;
};

// MPU6050 INIT
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

// READ MPU6050
IMUData readIMU() {
  IMUData d;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() < 14) {
    d.ax = d.ay = d.az = d.gy = 0;
    return d;
  }

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  int16_t raw_temp = (Wire.read() << 8) | Wire.read();
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  d.ax = (raw_ax / 16384.0) + OFFSET_AX;
  d.ay = (raw_ay / 16384.0) + OFFSET_AY;
  d.az = (raw_az / 16384.0) + OFFSET_AZ;
  d.gy = (raw_gy / 131.0) - OFFSET_GY;

  return d;
}

// KALMAN FILTER
float updateKalman(float new_angle, float new_rate, float dt) {
  float rate = new_rate - kalman_bias;
  kalman_angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = new_angle - kalman_angle;
  kalman_angle += K0 * y;
  kalman_bias  += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;

  return kalman_angle;
}

// SETUP
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Tăng tốc độ giao tiếp I2C lên 400kHz để đọc dữ liệu MPU6050 nhanh hơn
  Wire.setClock(400000); 

  mpuInit();
  delay(1000);

  IMUData data = readIMU();
  float init_pitch = atan2(-data.ax, sqrt(data.ay * data.ay + data.az * data.az)) * 180.0 / PI;

  comp_angle   = init_pitch;
  kalman_angle = init_pitch;

  last_time = micros();
  timer_plotter = millis();
}

// LOOP
void loop() {
  // Chạy lặp liên tục không block để tính dt chuẩn xác nhất
  unsigned long now = micros();
  float dt = (float)(now - last_time) / 1000000.0;
  
  if (dt <= 0.0 || dt > 0.1) dt = 0.01;
  last_time = now;

  IMUData data = readIMU();

  float accel_angle = atan2(-data.ax, sqrt(data.ay * data.ay + data.az * data.az)) * 180.0 / PI;

  // Cập nhật bộ lọc liên tục
  comp_angle = ALPHA * (comp_angle + data.gy * dt) + (1.0 - ALPHA) * accel_angle;
  kalman_angle = updateKalman(accel_angle, data.gy, dt);

  // Gửi dữ liệu lên Serial Plotter theo chu kỳ cố định hoàn hảo (Bỏ delay)
  if (millis() - timer_plotter >= SAMPLE_PERIOD_MS) {
    timer_plotter = millis();

    // In theo định dạng gắn nhãn (Label) giúp giao diện Serial Plotter nhận diện đúng tên đường
    Serial.print("Raw_Accel:");
    Serial.print(accel_angle);
    Serial.print(",");

    Serial.print("Complementary_Filter:");
    Serial.print(comp_angle);
    Serial.print(",");

    Serial.print("Kalman_Filter:");
    Serial.println(kalman_angle);
  }
}