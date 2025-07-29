#include <Arduino.h>

#define DEBUG 		1

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <AHTxx.h>

#include "ScioSense_ENS160.h"  // ENS160 library
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

#define SEALEVELPRESSURE_HPA (1013.25)

#define SoftI2C_SDA   8  
#define SoftI2C_SCL   7  
#define HardI2C_SDA  	5
#define HardI2C_SCL  	6
TwoWire Wire_Main = TwoWire(0); // I2C bus 0
TwoWire Wire_BMP 	= TwoWire(1); // I2C bus 1 

Adafruit_BMP280 bmp(&Wire_BMP); // Create BMP280 object with custom I2C bus
AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR, &Wire_Main); //sensor address, sensor type, TwoWire bus



float pressure, altitude, temperature, humidity; // Variables to hold sensor data

void readBMP280(){
	pressure = bmp.readPressure() / 100.0F;
	altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	if (DEBUG) {Serial.println("Pressure: " + String(pressure) + " hPa \t Altitude: " + String(altitude) + " m");}
}

void readaht21() {
	temperature = aht21.readTemperature();
	humidity = aht21.readHumidity();
	if (DEBUG) { Serial.println("Temperature: " + String(temperature) + " °C \t Humidity: " + String(humidity) + " %"); }
}


void setup() {
	Serial.begin(115200);

	// INITIALIZE SENSORS

	Wire_Main.begin(HardI2C_SDA, HardI2C_SCL); delay(100); // Initialize I2C with custom pins
	// Initialize Custom I2C communication for BMP280
  Wire_BMP.begin(SoftI2C_SDA, SoftI2C_SCL);  delay(100); // Initialize I2C with custom pins
	Wire.begin(SoftI2C_SDA, SoftI2C_SCL);  delay(100); // Initialize I2C with custom pins
	aht21.begin(); // Initialize AHT21 sensor with custom I2C bus
	bmp.begin(0x77); 													 // Initialize BMP280 sensor at address 0x77

  ens160.begin();
	
}

void loop() {
	readBMP280(); 		// Read Pressure sensor data
	readaht21();	 		// Read AHT21 sensor data
	delay(1000); 			// Wait for 1 second before the next reading
}


// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_BMP280.h>
// #include "AHTxx.h"  // Your patched version of the AHTxx library

// #define SEALEVELPRESSURE_HPA (1013.25)

// // Pin definitions for each I2C bus
// #define BMP_SDA 3
// #define BMP_SCL 2

// #define AHT_SDA 8
// #define AHT_SCL 7

// // Create TwoWire instances for each bus
// TwoWire Wire_BMP = TwoWire(1);  // I2C bus 1 for BMP280
// TwoWire Wire_AHT = TwoWire(0);  // I2C bus 0 for AHT21

// // Sensor objects, passing the corresponding TwoWire instances
// Adafruit_BMP280 bmp(&Wire_BMP);
// AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR, &Wire_AHT);

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   // Initialize both I2C buses with dedicated pins
//   Wire_BMP.begin(BMP_SDA, BMP_SCL);
//   Wire_AHT.begin(AHT_SDA, AHT_SCL);
//   delay(100);

//   Serial.println("\nInitializing BMP280 sensor...");

//   // Try typical BMP280 addresses
//   if (!bmp.begin(0x77)) {
//     Serial.println("BMP280 not found at 0x77, trying 0x76...");
//     if (!bmp.begin(0x76)) {
//       Serial.println("ERROR: BMP280 not found! Check wiring.");
//     } else {
//       Serial.println("BMP280 found at 0x76.");
//     }
//   } else {
//     Serial.println("BMP280 found at 0x77.");
//   }

//   Serial.println("\nInitializing AHT21 sensor...");
  
//   // Initialize AHT21 (patched library to accept custom TwoWire)
//   if (!aht21.begin()) {
//     Serial.println("ERROR: AHT21 sensor not found! Check wiring.");
//   } else {
//     Serial.println("AHT21 sensor initialized.");
//   }

//   Serial.println("\nSetup complete.\n");
// }

// void loop() {
//   // Read BMP280 sensor data
//   float pressure = bmp.readPressure() / 100.0F;  // hPa
//   float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

//   if (isnan(pressure) || isnan(altitude)) {
//     Serial.println("Failed to read BMP280 sensor!");
//   } else {
//     Serial.printf("Pressure: %.2f hPa \t Altitude: %.2f m\n", pressure, altitude);
//   }

//   // Read AHT21 sensor data
//   float temperature = aht21.readTemperature();
//   float humidity = aht21.readHumidity();

//   if (isnan(temperature) || isnan(humidity)) {
//     Serial.println("Failed to read AHT21 sensor!");
//   } else {
//     Serial.printf("Temperature: %.2f °C \t Humidity: %.2f %%\n", temperature, humidity);
//   }

//   Serial.println();
//   delay(2000);
// }

// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_BMP280.h>

// #define BMP_SDA 3
// #define BMP_SCL 2

// TwoWire Wire_BMP = TwoWire(1);
// Adafruit_BMP280 bmp(&Wire_BMP);

// void setup() {
// 	delay(5000);
//   Serial.begin(115200);
// 	Serial.println("Initializing...");
//   Wire_BMP.begin(BMP_SDA, BMP_SCL);
// 	Serial.println("Wire_BMP initialized.");
//   delay(100);
//   Serial.println("Initializing BMP280 sensor...");
//   if (!bmp.begin(0x77)) {
//     Serial.println("BMP280 not found at 0x77, trying 0x76...");
//     if (!bmp.begin(0x76)) {
//       Serial.println("BMP280 NOT DETECTED!");
//       while (1);
//     } else {
//       Serial.println("BMP280 detected at 0x76.");
//     }
//   } else {
//     Serial.println("BMP280 detected at 0x77.");
//   }
// }

// void loop() {
//   float pressure = bmp.readPressure() / 100.0F;
//   Serial.printf("Pressure: %.2f hPa\n", pressure);
//   delay(1000);
// }


// #include <Wire.h>

// #define BMP_SDA 3
// #define BMP_SCL 2

// TwoWire Wire_BMP = TwoWire(1);

// void setup() {
//   Serial.begin(115200);
// 	delay(5000); // Wait for Serial to initialize
//   Serial.println("Serial started");

//   Wire_BMP.begin(BMP_SDA, BMP_SCL);
//   Serial.println("Wire_BMP initialized");

//   Wire_BMP.beginTransmission(0x77);
//   uint8_t err = Wire_BMP.endTransmission();
//   Serial.print("I2C transmission status to 0x77: ");
//   Serial.println(err);
// }

// void loop() {}