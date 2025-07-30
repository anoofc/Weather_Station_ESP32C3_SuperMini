#define DEBUG 		0                         // 0 or 1 to turn OFF or ON Debugging

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <AHTxx.h>
#include <ScioSense_ENS160.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h> // Include Adafruit GFX library for graphics support
#include <Fonts/FreeSerifBold9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>

#define SEALEVELPRESSURE_HPA (1000.0)      // Sea Level Atmospheric Pressure in hPA

#define SoftI2C_SDA   8  
#define SoftI2C_SCL   7  
#define HardI2C_SDA  	5
#define HardI2C_SCL  	6

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


#define DISPLAY_UPDATE_INTERVAL 5000 // Update display every 1000 ms (1 second)
#define SENSOR_READ_INTERVAL    100  // Read sensors every 1000 ms (1 second)

TwoWire Wire_Main = TwoWire(0);         // I2C bus 0
TwoWire Wire_BMP 	= TwoWire(1);         // I2C bus 1 

Adafruit_BMP280 bmp(&Wire_BMP);                           // Create BMP280 object with custom I2C bus
AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR, &Wire_Main); // Create AHT21 object with Sensor address, sensor type, TwoWire bus
ScioSense_ENS160 ens160(&Wire_Main, ENS160_I2CADDR_1);    // Create ENS160 object with I2C Bus and Sensor Address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire_Main, -1);

// VARIABLES TO HOLD SENSOR DATA
float pressure, altitude, temperature, humidity;
uint8_t aqi;
uint16_t tvoc, eco2, aqi500;

uint8_t  showSensor = 0;          // Variable to control which sensor data to show on the display
uint32_t displayUpdateTimer = 0;  // Variable to store the last time the display was updated
uint32_t sensorReadTimer = 0;     // Variable to store the last time the sensors were read

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

void readENS160(){
  if (ens160.available()) {
    ens160.measure(true);
    ens160.measureRaw(true);
    aqi    = ens160.getAQI();
    tvoc   = ens160.getTVOC();
    eco2   = ens160.geteCO2();

    if (DEBUG){ Serial.println("AQI: " + String(aqi) + "\t TVOC: " + String(tvoc) + " ppb \t eCO2: " + String(eco2) + "ppm");}
  }
}

void updateDisplay_bmp() {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 2:1 pixel scale

  display.setCursor(10, 25);    // Start at top-left corner, y=20 for vertical centering
  display.setFont(&FreeSerifBold12pt7b); 
  display.print(pressure, 2);  display.setFont(&FreeSerifBold9pt7b); display.println(F(" hPa"));
  display.setCursor(10, 55);    // Move cursor down for next line
  display.setFont(&FreeSerifBold12pt7b); 
  display.print(altitude, 2);  display.setFont(&FreeSerifBold9pt7b); display.println(F(" m"));
  display.display();           // Show the text on the display
}

void updateDisplay_aht21() {
  display.clearDisplay();
  display.setTextSize(1);      // Normal 2:1 pixel scale

  display.setCursor(10, 25);    // Start at top-left corner, y=20 for vertical centering
  display.setFont(&FreeSerifBold12pt7b);
  display.print(temperature, 2);   display.setFont(&FreeSerifBold9pt7b); display.println(F(" °C"));
  display.setCursor(10, 55);    // Move cursor down for next line
  display.setFont(&FreeSerifBold12pt7b); 
  display.print(humidity, 2);   display.setFont(&FreeSerifBold9pt7b); display.println(F(" %"));
  display.display();           // Show the text on the display
}

void updateDisplay_ens160() {
  display.clearDisplay();
  display.setTextSize(1);      // Normal 2:1 pixel scale

  display.setCursor(5, 15);    // Start at top-left corner, y=20 for vertical centering
  display.setFont(&FreeSerifBold9pt7b); 
  display.print(F("AQI : ")); display.setFont(&FreeSerifBold12pt7b); display.println(aqi);          
  display.setCursor(5, 35);    // Move cursor down for next line
  display.setFont(&FreeSerifBold9pt7b); 
  display.println(F("eCO2:"));  
  display.setCursor(30, 60);    // Move cursor down for next line
  display.setFont(&FreeSerifBold12pt7b); display.print(eco2); display.setFont(&FreeSerifBold9pt7b); display.println(F(" ppm"));
  display.display();           // Show the text on the display
}

void setup() {
	if (DEBUG) { Serial.begin(115200); }     // Initialize Serial Communication for Debugging
	
  // Initialize Main I2C communication for ENS160, AHT21, and OLED Display
	Wire_Main.begin(HardI2C_SDA, HardI2C_SCL); delay(100); // Initialize I2C with custom pins
	// Initialize Custom I2C communication for BMP280
  Wire_BMP.begin(SoftI2C_SDA, SoftI2C_SCL);  delay(100); // Initialize I2C with custom pins

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Draw white text

  // INITIALIZE SENSORS
  aht21.begin();                             // Initialize AHT21 sensor with custom I2C bus
	bmp.begin(0x77); 													 // Initialize BMP280 sensor at address 0x77/
  ens160.begin();
  if (ens160.setMode(ENS160_OPMODE_STD)) { Serial.println("Set standard mode successfully."); }
}

void loop() {
  if (millis() - sensorReadTimer >= SENSOR_READ_INTERVAL) {
    sensorReadTimer = millis(); // Reset the timer
    readBMP280(); 		// Read Pressure sensor data
    readaht21();	 		// Read AHT21 sensor data
    readENS160();     // Read ENS160 Sensor data
    if (DEBUG){Serial.println("---------------------------------------------------");}   // to seperate bw new & last sensor data 

    if      (showSensor == 0) { updateDisplay_bmp(); } // Show BMP280 data
    else if (showSensor == 1) { updateDisplay_aht21(); } // Show AHT21 data
    else if (showSensor == 2) { updateDisplay_ens160(); } // Show ENS160 data
  }
  
  if (millis() - displayUpdateTimer >= DISPLAY_UPDATE_INTERVAL) {
    displayUpdateTimer = millis(); // Reset the timer
    showSensor++; // Increment the sensor index
    if (showSensor > 2) { showSensor = 0; } // Reset to 0 if it exceeds the number of sensors
  }
}