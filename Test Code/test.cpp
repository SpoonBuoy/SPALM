#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

#define SD_CS_PIN 0
#define FLASH_CS_PIN 1
#define BUZZER_PIN 2
#define SERVO1_PIN 3
#define SERVO2_PIN 4
#define VOLTAGE_PIN 6
#define SERVO_PINS {7, 8, 9, 10, 14, 15, 22, 23}
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 13
#define GREEN_LED_PIN 16
#define RED_LED_PIN 17
#define SDA_PIN 18
#define SCL_PIN 19
#define BLUE_LED_PIN 20
#define BMP280_MPU6050_PIN 23

Adafruit_BMP280 bmp;
MPU6050 mpu6050(Wire);
Servo servos[8];

void setup() {
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(FLASH_CS_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  SPI.begin();
  SD.begin(SD_CS_PIN);
  digitalWrite(FLASH_CS_PIN, HIGH);
  
  for (int i = 0; i < 8; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }

  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  bmp.begin(0x76);

  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(BLUE_LED_PIN, HIGH);
}

void loop() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);

  for (int i = 0; i < 8; i++) {
    servos[i].write(90);
    delay(500);
    servos[i].write(0);
    delay(500);
    servos[i].write(180);
    delay(500);
    servos[i].write(90);
    delay(500);
  }

  float voltage = analogRead(VOLTAGE_PIN) * 0.0033;
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  float temperature = bmp.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  float pressure = bmp.readPressure() / 100.0;
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  mpu6050.update();
  float roll = mpu6050.getAngleX();
  float pitch = mpu6050.getAngleY();
  float gyroX = mpu6050.getGyroX();
  float gyroY = mpu6050.getGyroY();
  float gyroZ = mpu6050.getGyroZ();
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.println(" degrees");

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.println(" degrees");

