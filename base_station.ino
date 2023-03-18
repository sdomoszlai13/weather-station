#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>
#include <NRFLite.h>


// Instantiate an AHT20 sensor
AHT20 aht20;

// Instantiate a BMP280 sensor
Adafruit_BMP280 bmp280;    // use I2C interface
Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();

// Instantiate a display
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Instantiate an nRF24L01 transceiver


void setup(){

    Serial.begin(115200);

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


void receive(float temp, int hum, int pres){
    // temp = 
    // hum = 
    // pres =
}
