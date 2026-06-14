#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68
#define NUM_SAMPLES 1000

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

void readMPU(
  int16_t &ax,
  int16_t &ay,
  int16_t &az,
  int16_t &gy)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  Wire.read();
  Wire.read();

  gy = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();
}

void setup() {

  Serial.begin(115200);
  Wire.begin();

  mpuInit();

  delay(3000);

  Serial.println("GIU YEN CAM BIEN");

  double meanAngle = 0;
  double meanGyro = 0;

  double M2Angle = 0;
  double M2Gyro = 0;

  float gyroFirst = 0;
  float gyroLast = 0;

  for (int n = 1; n <= NUM_SAMPLES; n++) {

    int16_t ax, ay, az, gy;

    readMPU(ax, ay, az, gy);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gyro = gy / 131.0;

    float angle =
      atan2(
        -ax_g,
        sqrt(ay_g * ay_g + az_g * az_g)
      ) * 180.0 / PI;

    if (n == 1)
      gyroFirst = gyro;

    gyroLast = gyro;

    double deltaA = angle - meanAngle;
    meanAngle += deltaA / n;
    M2Angle += deltaA * (angle - meanAngle);

    double deltaG = gyro - meanGyro;
    meanGyro += deltaG / n;
    M2Gyro += deltaG * (gyro - meanGyro);

    delay(10);
  }

  float varAccel = M2Angle / (NUM_SAMPLES - 1);
  float varGyro  = M2Gyro / (NUM_SAMPLES - 1);

  float stdAccel = sqrt(varAccel);
  float stdGyro  = sqrt(varGyro);

  float drift = fabs(gyroLast - gyroFirst);

  float R_measure = varAccel;
  float Q_angle   = varGyro;
  float Q_bias    = (drift * drift) / 10.0;

  Serial.println();
  Serial.println("===== KET QUA =====");

  Serial.print("Std Accel = ");
  Serial.println(stdAccel, 6);

  Serial.print("Std Gyro = ");
  Serial.println(stdGyro, 6);

  Serial.println();

  Serial.print("R_measure = ");
  Serial.println(R_measure, 6);

  Serial.print("Q_angle = ");
  Serial.println(Q_angle, 6);

  Serial.print("Q_bias = ");
  Serial.println(Q_bias, 6);

  Serial.println("===================");
}

void loop() {
}