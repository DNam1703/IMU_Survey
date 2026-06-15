#include <Wire.h>
#include <math.h>

#define MPU_ADDR           0x68
#define SAMPLE_RATE_HZ     100      

// Các biến lưu trữ dữ liệu thô và giá trị offset sau đo đạc
float ax_off = 0.0f, ay_off = 0.0f, az_off = 0.0f; [cite: 4]
float gx_off = 0.0f, gy_off = 0.0f, gz_off = 0.0f; [cite: 4]

int16_t raw_ax, raw_ay, raw_az; [cite: 5]
int16_t raw_gx, raw_gy, raw_gz; [cite: 5]
bool system_ready = false; [cite: 5]

// Hàm giao tiếp I2C đọc trực tiếp dữ liệu thô từ cảm biến MPU6050
void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false); [cite: 15]
  Wire.requestFrom(MPU_ADDR, 14, true); [cite: 15]
  if (Wire.available() >= 14) { [cite: 16]
    raw_ax = Wire.read()<<8 | Wire.read(); raw_ay = Wire.read()<<8 | Wire.read(); [cite: 16]
    raw_az = Wire.read()<<8 | Wire.read(); [cite: 17]
    Wire.read(); Wire.read(); // Bỏ qua thanh ghi nhiệt độ thô [cite: 17]
    raw_gx = Wire.read()<<8 | Wire.read(); raw_gy = Wire.read()<<8 | Wire.read(); [cite: 17]
    raw_gz = Wire.read()<<8 | Wire.read(); [cite: 18]
  }
}

// Khởi tạo các cấu hình phần cứng cho MPU6050
void initMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission(); delay(50); [cite: 18]
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); [cite: 18]
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); [cite: 18, 19]
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); [cite: 19]
}

// ────────────────────────────────────────────────────────────────────────────
//  BƯỚC 1: TỰ ĐỘNG ĐO ĐẠC VÀ HIỆU CHỈNH BIẾN OFFSET ĐỘNG (6 VỊ TRÍ CHUẨN)
// ────────────────────────────────────────────────────────────────────────────
void runCalibrationRoutine() {
  Serial.println(F("\n[2.2] Bat dau do 500 mau Gyro Offset... GIU YEN LANG...")); [cite: 20]
  long sum_gx = 0, sum_gy = 0, sum_gz = 0; [cite: 20]
  
  for (int i = 0; i < 500; i++) { [cite: 21]
    readMPU(); [cite: 21]
    sum_gx += raw_gx; sum_gy += raw_gy; [cite: 21]
    sum_gz += raw_gz; [cite: 22]
    delay(6); [cite: 22]
  }
  // Tính trung bình trôi dịch của con quay hồi chuyển Gyroscope [cite: 22]
  gx_off = (sum_gx / 500.0f) / 131.0f; [cite: 22]
  gy_off = (sum_gy / 500.0f) / 131.0f; [cite: 22]
  gz_off = (sum_gz / 500.0f) / 131.0f; [cite: 23]

  Serial.println(F("\n[2.1] Quy trinh do 6 vi tri chuan cho Accelerometer...")); [cite: 23]
  float m_az_up = 0, m_az_down = 0, m_ax_up = 0, m_ax_down = 0, m_ay_up = 0, m_ay_down = 0; [cite: 24]
  
  const char* prompts[] = { [cite: 25]
    "Z huong thang dung LEN TROI (+1g) [Ngua phang]", 
    "Z huong thang dung XUONG DAT (-1g) [Up phang]", 
    "X huong thang dung LEN TROI (+1g) [Dung dung]", 
    "X huong thang dung XUONG DAT (-1g) [Dung nguoc]", 
    "Y huong thang dung LEN TROI (+1g) [Nghieng trai]", 
    "Y huong thang dung XUONG DAT (-1g) [Nghieng phai]" 
  }; [cite: 25]
  
  float* targets[] = { &m_az_up, &m_az_down, &m_ax_up, &m_ax_down, &m_ay_up, &m_ay_down }; [cite: 26]
  
  for (int step = 0; step < 6; step++) { [cite: 27]
    while(Serial.available() > 0) { Serial.read(); } [cite: 27, 28]
    
    Serial.println(F("----------------------------------------------------------------")); [cite: 28]
    Serial.print(F("DAT CAM BIEN VI TRI: ")); Serial.println(prompts[step]); [cite: 28]
    Serial.println(F(">> Go chu 'g' roi Enter de do...")); [cite: 29]
    
    // Chờ ký tự xác nhận lệnh từ bàn phím [cite: 29]
    while (true) { 
      if (Serial.available() > 0) { [cite: 29]
        char ch = Serial.read(); [cite: 29]
        if (ch == 'g' || ch == 'G') break; [cite: 30]
      }
    }
    
    Serial.print(F(">> Cho on dinh 2 giay...")); [cite: 31]
    for(int s = 0; s < 20; s++) { readMPU(); delay(100); } [cite: 31]
    Serial.println(F(" BAT DAU DO!")); [cite: 31]
    
    double sum = 0; [cite: 32]
    for (int i = 0; i < 200; i++) { [cite: 32]
      readMPU(); [cite: 32]
      if (step == 0 || step == 1) sum += (raw_az / 16384.0f); [cite: 33]
      if (step == 2 || step == 3) sum += (raw_ax / 16384.0f); [cite: 34]
      if (step == 4 || step == 5) sum += (raw_ay / 16384.0f); [cite: 35]
      delay(10); [cite: 35]
    }
    *targets[step] = sum / 200.0f; [cite: 36]
    Serial.print(F(" -> Hoan thanh vi tri ")); Serial.print(step + 1); [cite: 36]
    Serial.print(F(". Mean G = ")); Serial.println(*targets[step], 4); [cite: 37]
  }

  // Thuật toán tính toán bù sai số động từ kết quả đo 6 hướng [cite: 37]
  ax_off = -((m_ax_up + m_ax_down) / 2.0f); [cite: 37]
  ay_off = -((m_ay_up + m_ay_down) / 2.0f); [cite: 38]
  az_off = -((m_az_up + m_az_down) / 2.0f); [cite: 38]
}

// ────────────────────────────────────────────────────────────────────────────
//  BƯỚC 2: THAO TÁC THỦ CÔNG ĐỂ BẮT ĐẦU XÁC THỰC 100 MẪU
// ────────────────────────────────────────────────────────────────────────────
void runValidationRoutine() {
  while(Serial.available() > 0) { Serial.read(); } [cite: 39, 40]

  Serial.println(F("\n====================================================================")); [cite: 40]
  Serial.println(F(" [CAU HINH XAC THUC 100 MAU THU CONG]")); [cite: 40]
  Serial.println(F(" >> Dat MPU6050 nam ngua, phang tren mat ban tinh.")); [cite: 41]
  Serial.println(F(" >> SAU DO: Go phim 's' va nhan Enter de quet mau!")); [cite: 42]
  Serial.println(F("====================================================================")); [cite: 42]
  
  while (true) { [cite: 43]
    if (Serial.available() > 0) { [cite: 43]
      char ch = Serial.read(); [cite: 43]
      if (ch == 's' || ch == 'S') break; [cite: 44]
    }
  }

  Serial.print(F(">> Kich hoat...")); [cite: 45]
  for (int j = 0; j < 10; j++) { readMPU(); delay(100); } [cite: 45]
  Serial.println(F(" BAT DAU QUET!")); [cite: 45]
  
  double check_sum_magnitude = 0; [cite: 46]
  for (int i = 0; i < 100; i++) { [cite: 46]
    readMPU(); [cite: 46]
    float Ax_cal = (raw_ax / 16384.0f) + ax_off; [cite: 47]
    float Ay_cal = (raw_ay / 16384.0f) + ay_off; [cite: 47]
    float Az_cal = (raw_az / 16384.0f) + az_off; [cite: 48]
    check_sum_magnitude += sqrtf(Ax_cal*Ax_cal + Ay_cal*Ay_cal + Az_cal*Az_cal); [cite: 48]
    delay(10); [cite: 48]
  }

  float final_mean_magnitude = check_sum_magnitude / 100.0f; [cite: 49]
  if (fabsf(final_mean_magnitude - 1.0f) < 0.15f) { [cite: 49]
    Serial.println(F("\n>> THOA MAN! HE THONG SAN SANG.\n")); [cite: 49]
    system_ready = true; [cite: 50]
  } else {
    Serial.println(F("\n>> [THAT BAI] KIEM TRA LAI DO PHANG VA RESET!")); [cite: 50]
    system_ready = false; [cite: 51]
    while(1); // Dừng hệ thống nếu không đạt kiểm tra sai số ổn định 
  }
}

void setup() {
  Serial.begin(115200); [cite: 51]
  while(!Serial); [cite: 51]
  Wire.begin(); [cite: 51]
  Wire.setClock(400000); // Tăng tốc độ giao tiếp bus I2C lên 400kHz 

  initMPU();             // Cấu hình thanh ghi MPU6050 
  runCalibrationRoutine(); // Thực hiện đo 6 hướng thu thập offset 
  runValidationRoutine();  // Chạy xác thực độ chuẩn 
}

void loop() {
  if (!system_ready) return; [cite: 55]

  // Khóa luồng in liên tục, hiển thị các hằng số cấu hình tĩnh thu được
  Serial.println(F("\n=========================================================="));
  Serial.println(F("             DANH SACH BIEN OFFSET HOAN THANH             "));
  Serial.println(F("=========================================================="));
  
  Serial.print(F("-> ax_off = ")); Serial.print(ax_off, 6); Serial.println(F("f;"));
  Serial.print(F("-> ay_off = ")); Serial.print(ay_off, 6); Serial.println(F("f;"));
  Serial.print(F("-> az_off = ")); Serial.print(az_off, 6); Serial.println(F("f;"));
  
  Serial.print(F("-> gx_off = ")); Serial.print(gx_off, 6); Serial.println(F("f;"));
  Serial.print(F("-> gy_off = ")); Serial.print(gy_off, 6); Serial.println(F("f;"));
  Serial.print(F("-> gz_off = ")); Serial.print(gz_off, 6); Serial.println(F("f;"));
  Serial.println(F("=========================================================="));
  Serial.println(F(">> copy cac gia tri tren de thay vao cac chuong trinh chay logic."));

  // Treo chương trình sau khi kết thúc xuất kết quả để tránh lặp bộ đệm Serial
  while(1) { delay(1000); }
}