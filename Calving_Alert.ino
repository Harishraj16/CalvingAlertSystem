#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SoftwareSerial.h>

// === Pin Definitions ===
SoftwareSerial sim900(10, 11); // RX, TX for SIM900A

// === Sensor Setup ===
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const int MPU_ADDR = 0x68; // MPU6050 I2C address

// === Thresholds ===
#define TEMP_THRESHOLD 35.0 // Celsius
#define ACC_X_THRESHOLD 10000
#define ACC_Y_THRESHOLD 8000

// === Variables ===
int16_t acc_x, acc_y, acc_z;
bool alertSent = false;

void setup() {
  Serial.begin(9600);
  sim900.begin(9600);

  Wire.begin();
  mlx.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);

  delay(1000);
}

void loop() {
  readAccelerometer();
  float objectTemp = mlx.readObjectTempC();

  Serial.print("Accel X: "); Serial.print(acc_x);
  Serial.print(" | Accel Y: "); Serial.print(acc_y);
  Serial.print(" | Accel Z: "); Serial.print(acc_z);
  Serial.print(" | Temp: "); Serial.println(objectTemp);

  if (!alertSent &&
      (abs(acc_x) > ACC_X_THRESHOLD || abs(acc_y) > ACC_Y_THRESHOLD) &&
      objectTemp < TEMP_THRESHOLD) {
    sendSMS();
    makeCall();
    alertSent = true; // Prevent repeated alerts
  }

  delay(1000); // Check every second
}

void readAccelerometer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6); // Read ACC_X, ACC_Y, ACC_Z

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
}

void sendSMS() {
  sim900.println("AT+CMGF=1"); // Text mode
  delay(1000);
  sim900.println("AT+CMGS=\"+91XXXXXXXXXX\""); // Replace with actual number
  delay(1000);
  sim900.print("ALERT: Calving may have started. Please check your cow immediately.");
  delay(500);
  sim900.write(26); // CTRL+Z to send
  delay(5000);
}

void makeCall() {
  sim900.println("ATD+91XXXXXXXXXX;"); // Replace with actual number
  delay(20000); // Let it ring for 20 seconds
  sim900.println("ATH"); // Hang up
}
