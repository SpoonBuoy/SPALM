#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

const int motorPin1 = 2;  // Connect motor pin 1 to Teensy digital pin 2
const int motorPin2 = 3;  // Connect motor pin 2 to Teensy digital pin 3

// PID Constants
const float Kp = 1.0;
const float Ki = 0.2;
const float Kd = 0.1;

// PID Variables
float setpoint = 0;
float input, output;
float error, lastError;
float integral, derivative;

// Motor Control Limits
const int motorMin = 0;
const int motorMax = 255;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  while (!Serial)
    ;

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {
  mpu6050.update();

  float roll = mpu6050.getAngleX();
  float pitch = mpu6050.getAngleY();

  input = roll; // Use roll angle as input for PID controller

  // PID Calculation
  error = setpoint - input;
  integral += error;
  derivative = error - lastError;

  output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, motorMin, motorMax);

  // Apply thrust vector to motors
  analogWrite(motorPin1, output);
  analogWrite(motorPin2, output);

  lastError = error;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\tOutput: ");
  Serial.println(output);

  delay(10);
}
