#include <Arduino.h>

#define DEBUG 		1

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bmp;

#define I2C_SDA   3  
#define I2C_SCL   2  
float pressure, altitude; // Variables to hold sensor data

void readBMP280(){
	pressure = bmp.readPressure() / 100.0F;
	altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	if (DEBUG) {Serial.println("Pressure: " + String(pressure) + " hPa \t Altitude: " + String(altitude) + " m");}
}

void setup() {
	Serial.begin(115200);

	// Initialize Custom I2C communication for BMP280
  Wire.begin(I2C_SDA, I2C_SCL);  delay(100); // Initialize I2C with custom pins
	bmp.begin(0x77); 													 // Initialize BMP280 sensor at address 0x77
}

void loop() {
	readBMP280(); // Read Pressure sensor data
	delay(500); // Wait for 1 second before the next reading
}