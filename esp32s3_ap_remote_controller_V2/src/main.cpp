#include <Arduino.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "secrets.h"

const int BMI160_ADDR = 0x69;
const int JOYSTICK_ADDR = 0x63;

// ===== WiFi / UDP Configuration =====
static const char *AP_SSID = AP_SSID_VAL;
static const char *AP_PASS = AP_PASS_VAL;
static const char *PICO_IP = PICO_IP_VAL;
static const uint16_t TELE_PORT = TELE_PORT_VAL;

WiFiUDP udp;
static const float MAX_SPEED = 0.4f;
static const float MAX_TURN = 0.2f;

float g_speed = 0.0f;
float g_turn = 0.0f;
float g_gx = 0.0f, g_gy = 0.0f, g_gz = 0.0f; // Global gyro values for telemetry

void sendCommand(float speed, float turn, float q0, float q1, float q2, float q3, float gx, float gy, float gz) {
  char buf[96];
  snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f", 
           speed, turn, q0, q1, q2, q3, gx, gy, gz);
  udp.beginPacket(PICO_IP, CMD_PORT_VAL);
  udp.print(buf);
  udp.endPacket();
}

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float twoKp = 2.0f;
float twoKi = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
uint32_t lastUpdate = 0;

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void initBMI160() {
  Wire.begin(8, 9);
  Wire.setClock(400000);
  delay(100);
  
  writeRegister(0x7E, 0xB6); // Soft reset
  delay(100);
  
  // Power up Accel and Gyro to Normal mode
  writeRegister(0x7E, 0x11); // Accel normal
  delay(100);
  writeRegister(0x7E, 0x15); // Gyro normal
  delay(100); // BMI160 needs ~80ms to start gyro clock
  
  writeRegister(0x41, 0x05); // +-4g
  writeRegister(0x40, 0x28); // 100Hz ODR
  writeRegister(0x43, 0x01); // 1000 dps
  writeRegister(0x42, 0x28); // 100Hz ODR
  delay(50);
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
                         float az, float dt) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Setup WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  udp.begin(TELE_PORT); 
  Serial.printf("[AP] IP: %s\n", WiFi.softAPIP().toString().c_str());

  // Initialize BMI160
  initBMI160(); // Uses Wire (SDA=8, SCL=9)
  
  // Quick I2C check
  Wire.beginTransmission(BMI160_ADDR);
  if (Wire.endTransmission() == 0) {
      Serial.println("[OK] BMI160 detected at 0x69");
  } else {
      Serial.println("[ERR] BMI160 not found at 0x69!");
  }

  // Initialize Joystick (Using Wire1 on SDA=4, SCL=5)
  Wire1.begin(4, 5); 
  Wire1.setClock(400000);
  
  Wire1.beginTransmission(JOYSTICK_ADDR);
  if (Wire1.endTransmission() == 0) {
      Serial.println("[OK] Joystick detected at 0x63");
  } else {
      Serial.println("[ERR] Joystick not found at 0x63 on Wire1!");
  }
  
  lastUpdate = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)BMI160_ADDR, (uint8_t)12, (uint8_t) true);

  if (Wire.available() >= 12) {
    uint8_t buf[12];
    for (int i = 0; i < 12; i++)
      buf[i] = Wire.read();

    int16_t gx_raw = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t gy_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t gz_raw = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t ax_raw = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t ay_raw = (int16_t)(buf[8] | (buf[9] << 8));
    int16_t az_raw = (int16_t)(buf[10] | (buf[11] << 8));

    g_gx = gx_raw / 32.8f;
    g_gy = gy_raw / 32.8f;
    g_gz = gz_raw / 32.8f;
    float ax = ax_raw / 8192.0f;
    float ay = ay_raw / 8192.0f;
    float az = az_raw / 8192.0f;

    float gx_rad = g_gx * 0.0174533f;
    float gy_rad = g_gy * 0.0174533f;
    float gz_rad = g_gz * 0.0174533f;

    MahonyAHRSupdateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az, dt);
  }

  // --- Read Joystick over Wire1 (I2C addr 0x63) ---
  Wire1.beginTransmission(JOYSTICK_ADDR);
  Wire1.write(0x60); // JOYSTICK2_OFFSET_ADC_VALUE_8BITS_REG
  Wire1.endTransmission(false);
  Wire1.requestFrom((uint16_t)JOYSTICK_ADDR, (uint8_t)2, (uint8_t)true);
  
  if (Wire1.available() >= 2) {
    int8_t x_offset = Wire1.read(); // -128 to 127
    int8_t y_offset = Wire1.read(); // -128 to 127

    // Handle small deadzone
    if (abs(x_offset) < 10) x_offset = 0;
    if (abs(y_offset) < 10) y_offset = 0;

    // Map joystick offsets to g_speed and g_turn
    // Typically UP is negative Y (if hardware oriented conventionally)
    // Please verify the sign with the actual behavior.
    g_speed = -(float)y_offset / 128.0f * MAX_SPEED;
    g_turn = -(float)x_offset / 128.0f * MAX_TURN; 
  }

  static uint32_t lastSendMs = 0;
  // Send UDP every 25ms (40Hz)
  if (millis() - lastSendMs > 25) {
    lastSendMs = millis();
    sendCommand(g_speed, g_turn, q0, q1, q2, q3, g_gx, g_gy, g_gz);
  }

  // Check incoming telemetry (Optional, like original V1 remote)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[64];
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      // You can parse telemetry if needed
    }
  }

  delay(10);
}
