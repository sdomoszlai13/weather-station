#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <RF24.h>
#include <string.h>

#define SCREEN_WIDTH 128    // OLED display width (in pixels)
#define SCREEN_HEIGHT 64    // OLED display height (in pixels)
#define OLED_RESET -1    // Reset pin number (-1: sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C    // Display address; 0x3D for 128 X 64, 0x3C for 128 X 32

// Instantiate an AHT20 sensor
AHT20 aht20;

// Instantiate a BMP280 sensor
Adafruit_BMP280 bmp280;    // use I2C interface
Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();

// Instantiate a display
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Instantiate a display
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Instantiate an nRF24L01 transceiver
RF24 radio(7, 8);   // Using pin 7 as the CE pin, and pin 8 as the CSN pin

void setup(){

    Serial.begin(115200);

    /*
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);    // Loop forever
    }

    display.clearDisplay();    // Clear buffer
    */

    Wire.begin();
    aht20.begin();

    if (!bmp280.begin()){
        Serial.println("Could not find a valid BMP280 sensor, check wiring"
                        " or try a different address!");
        while (1) delay(500);
        Serial.println("Setup finished");
    }

    // Default settings from datasheet
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,   // Temperature oversampling
                    Adafruit_BMP280::SAMPLING_X16,  // Pressure oversampling
                    Adafruit_BMP280::FILTER_X16,    // Filtering
                    Adafruit_BMP280::STANDBY_MS_500);   // Standby time
    
    lcd.init();
    lcd.backlight();
}


void loop(){

    // AHT20 measurements
    float aht20hum = aht20.getHumidity();
    float aht20temp = aht20.getTemperature();

    // BMP280 measurements
    float bmp280temp = bmp280.readTemperature();
    float bmp280pres = bmp280.readPressure();

    delay(50);

    // Print values on display
    lcd.setCursor(1,0);
    lcd.print("Temp: ");
    lcd.setCursor(7,0);
    lcd.print(bmp280temp);
    lcd.setCursor(1,1);
    lcd.print("Pres: ");
    lcd.setCursor(7,1);
    bmp280pres = bmp280pres / 100;
    lcd.print(bmp280pres);
    delay(1000);
}

/*
void printData(int hum, float temp) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);    // Draw white text
    display.setCursor(0,0);    // Start at top left corner

    display.print("Relative humidity: ");
    display.print(hum);
    display.print("%");
    display.print("\n");
    display.print("Temperature: ");
    display.print(temp, 1);

    display.display();
    delay(400);
}
*/


void receive(float temp, int hum, int pres){
    // temp = 
    // hum = 
    // pres =
}
