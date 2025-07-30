#define DEBUG 		1                         // 0 or 1 to turn OFF or ON Debugging

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <AHTxx.h>
#include <ScioSense_ENS160.h>
// #include <u8g2lib.h>

#include <Adafruit_SSD1306.h>


#define SEALEVELPRESSURE_HPA (1013.25)      // Sea Level Atmospheric Pressure in hPA

#define SoftI2C_SDA   8  
#define SoftI2C_SCL   7  
#define HardI2C_SDA  	5
#define HardI2C_SCL  	6

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

TwoWire Wire_Main = TwoWire(0);         // I2C bus 0
TwoWire Wire_BMP 	= TwoWire(1);         // I2C bus 1 

Adafruit_BMP280 bmp(&Wire_BMP);                           // Create BMP280 object with custom I2C bus
AHTxx aht21(AHTXX_ADDRESS_X38, AHT2x_SENSOR, &Wire_Main); // Create AHT21 object with Sensor address, sensor type, TwoWire bus
ScioSense_ENS160 ens160(&Wire_Main, ENS160_I2CADDR_1);    // Create ENS160 object with I2C Bus and Sensor Address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire_Main, -1);


// U8G2_SSD1306_128X64_NONAME_1_SW_I2C display(U8G2_R0, /* clock=*/ HardI2C_SCL, /* data=*/ HardI2C_SDA, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// VARIABLES TO HOLD SENSOR DATA
float pressure, altitude, temperature, humidity;
uint8_t aqi;
uint16_t tvoc, eco2, aqi500;


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

// void updateDisplay(){
//   display.clearBuffer();					// clear the internal memory
//   display.setFont(u8g2_font_ncenB12_tr);	// choose a suitable font
//   display.drawStr(0,30,"Hello World!");	// write something to the internal memory
//   display.sendBuffer();					// transfer internal memory to the display
// }

void updateDisplay() {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 2:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 30);    // Start at top-left corner, y=20 for vertical centering
  display.println(F("Hello World!"));

  display.display();           // Show the text on the display
}


void setup() {
	if (DEBUG) { Serial.begin(115200); }     // Initialize Serial Communication for Debugging
	
  // Initialize Main I2C communication for ENS160, AHT21, and OLED Display
	Wire_Main.begin(HardI2C_SDA, HardI2C_SCL); delay(100); // Initialize I2C with custom pins
	// Initialize Custom I2C communication for BMP280
  Wire_BMP.begin(SoftI2C_SDA, SoftI2C_SCL);  delay(100); // Initialize I2C with custom pins

	// display.begin(); // Initialize U8G2 with the main I2C bus
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // INITIALIZE SENSORS
  aht21.begin();                             // Initialize AHT21 sensor with custom I2C bus
	bmp.begin(0x77); 													 // Initialize BMP280 sensor at address 0x77/
  ens160.begin();
  if (ens160.setMode(ENS160_OPMODE_STD)) { Serial.println("Set standard mode successfully."); }
}

void loop() {
	readBMP280(); 		// Read Pressure sensor data
	readaht21();	 		// Read AHT21 sensor data
  readENS160();     // Read ENS160 Sensor data
  if (DEBUG){Serial.println("---------------------------------------------------");}   // to seperate bw new & last sensor data 
  delay(1000); 
  updateDisplay();  // Update the OLED display with the latest data
	delay(1000); 			// Wait for 1 second before the next reading

}

// #include <Arduino.h>
// #include <Wire.h>
// #include <U8g2lib.h>


// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// void setup(void) {
//   u8g2.begin();
// }

// void loop(void) {
//   u8g2.clearBuffer();					// clear the internal memory
//   u8g2.setFont(u8g2_font_ncenB12_tr);	// choose a suitable font
//   u8g2.drawStr(0,30,"Hello World!");	// write something to the internal memory
//   u8g2.sendBuffer();					// transfer internal memory to the display
//   delay(1000);  
// }