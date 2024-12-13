#include <Wire.h>
#include <ESP32Servo.h>

// Function declaration
void calibrateMPU();

// Define MPU6050 I2C address
#define MPU 0x68

// Servo objects
Servo servoX, servoY;

// Servo pins
#define SERVO_X_PIN 18
#define SERVO_Y_PIN 19

// Servo limits
const int SERVO_MIN = 30;
const int SERVO_MAX = 120;

// MPU6050 variables
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float roll, pitch;
float elapsedTime, currentTime, previousTime;

// PID variables
float Kp = 3.0, Ki = 0.5, Kd = 0.1; // Tuning parameters
float errorX, errorY, prevErrorX, prevErrorY, integralX, integralY;
float outputX, outputY;

// Calibration offsets
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;

// Servo positions
int servoPosX = 90; // Initial servo position
int servoPosY = 90; // Initial servo position

void setup() {
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Wake up MPU6050
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Attach servos
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Set initial servo positions
  servoX.write(servoPosX);
  servoY.write(servoPosY);

  // Calibrate MPU6050
  calibrateMPU();
}

void loop() {
  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (bool)true); // Fixed ambiguity

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope data === //
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Start with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (bool)true); // Fixed ambiguity

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroErrorX;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroErrorY;

  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;

  // Complementary filter to combine accelerometer and gyroscope data
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // === PID Control for X-axis === //
  errorX = -roll; // Target roll is 0°
  integralX += errorX * elapsedTime;
  float derivativeX = (errorX - prevErrorX) / elapsedTime;
  outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
  prevErrorX = errorX;

  // Adjust servo position for X-axis
  servoPosX = constrain(90 + outputX, SERVO_MIN, SERVO_MAX);
  servoX.write(servoPosX);

  // === PID Control for Y-axis === //
  errorY = -pitch; // Target pitch is 0°
  integralY += errorY * elapsedTime;
  float derivativeY = (errorY - prevErrorY) / elapsedTime;
  outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;
  prevErrorY = errorY;

  // Adjust servo position for Y-axis
  servoPosY = constrain(90 + outputY, SERVO_MIN, SERVO_MAX);
  servoY.write(servoPosY);

  delay(10); // Small delay for stability
}

void calibrateMPU() {
  int c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (bool)true); // Fixed ambiguity

    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    AccErrorX += (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    AccErrorY += (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);

    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (bool)true); // Fixed ambiguity

    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;

    GyroErrorX += GyroX;
    GyroErrorY += GyroY;

    c++;
  }

  AccErrorX /= 200;
  AccErrorY /= 200;
  GyroErrorX /= 200;
  GyroErrorY /= 200;
}
