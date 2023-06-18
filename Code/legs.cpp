#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>

Adafruit_BMP280 bmp;
Servo leg1, leg2, leg3, leg4;

const int bmpAltitudeThreshold = 10;  // Altitude threshold for deploying landing gear in meters

void setup() {
  Wire.begin();
  Serial.begin(9600);

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  leg1.attach(2);  // Connect leg1 servo to Teensy digital pin 2
  leg2.attach(3);  // Connect leg2 servo to Teensy digital pin 3
  leg3.attach(4);  // Connect leg3 servo to Teensy digital pin 4
  leg4.attach(5);  // Connect leg4 servo to Teensy digital pin 5

  leg1.write(0);   // Initialize leg1 servo position to 0 degrees
  leg2.write(0);   // Initialize leg2 servo position to 0 degrees
  leg3.write(0);   // Initialize leg3 servo position to 0 degrees
  leg4.write(0);   // Initialize leg4 servo position to 0 degrees
}

void loop() {
  float altitude = bmp.readAltitude();  // Read current altitude from BMP280 sensor

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");

  if (altitude >= bmpAltitudeThreshold) {
    deployLandingGear();  // Deploy landing gear if altitude exceeds threshold
  }

  delay(1000);
}

void deployLandingGear() {
  leg1.write(90);  // Set leg1 servo position to 90 degrees
  leg2.write(90);  // Set leg2 servo position to 90 degrees
  leg3.write(90);  // Set leg3 servo position to 90 degrees
  leg4.write(90);  // Set leg4 servo position to 90 degrees

  delay(2000);  // Wait for 2 seconds to allow landing gear deployment

  leg1.detach();  // Detach leg1 servo to hold its position
  leg2.detach();  // Detach leg2 servo to hold its position
  leg3.detach();  // Detach leg3 servo to hold its position
  leg4.detach();  // Detach leg4 servo to hold its position

  while (1);  // Stop further execution
}
