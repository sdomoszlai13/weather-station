#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>
#include <NRFLite.h>


// BASE STATION


// Instantiate an AHT20 sensor
AHT20 aht20;

// Instantiate a BMP280 sensor
Adafruit_BMP280 bmp280; // Use I2C interface
Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();

// Instantiate a display
LiquidCrystal_I2C lcd(0x27,20,4);   // Set address to 0x27 for a 16 X 2 display

// Instantiate a transceiver
NRFLite _radio;
const static uint8_t RADIO_ID = 0;              // This transceiver
const static uint8_t DESTINATION_RADIO_ID = 1;  // Other transceiver
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket  // Packet to be received
{
    float temp;
    float pres;
    float hum;
};

enum TypeOfVal {temp, pres, hum};   // Determines which display function to use


void setup(){

    Serial.begin(115200);
    Wire.begin();

    // Setup AHT20 sensor
    if (!aht20.begin())
    {
        Serial.println("Could not find a valid AHT20 sensor, check wiring"
                       " or try a different address!");
        while (1) delay(500);
    }

    Serial.println("AHT20 setup finished");

    // Setup BMP280 sensor
    if (!bmp280.begin())
    {
        Serial.println("Could not find a valid BMP280 sensor, check wiring"
                        " or try a different address!");
        while (1) delay(500);
    }

    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,       // Temperature oversampling
            Adafruit_BMP280::SAMPLING_X16,      // Pressure oversampling
            Adafruit_BMP280::FILTER_X16,        // Filtering
            Adafruit_BMP280::STANDBY_MS_500);   // Standby time
    Serial.println("BMP280 setup finished");
    
    // Setup display
    lcd.init();
    lcd.backlight();

    Serial.println("SSD1306 setup finished");

    // Setup transceiver
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial.println("Can't communicate with radio!");
        while (1) delay(500);
    }

    Serial.println("nRF24L01 setup finished");
}


void loop(){

    // AHT20 measurements
    float aht20hum = aht20.getHumidity();
    float aht20temp = aht20.getTemperature();

    // BMP280 measurements
    float bmp280temp = bmp280.readTemperature();
    float bmp280pres = bmp280.readPressure();

    // Receive values
    RadioPacket outdoor_data = receiveData();

    RadioPacket indoor_data;
    indoor_data.temp = bmp280temp;
    indoor_data.pres = bmp280pres / 100;    // Pressure in hPa
    indoor_data.hum = aht20hum;

    // Print temperatures on display
    printData(indoor_data.temp, outdoor_data.temp, temp);
    delay(4000);
    lcd.clear();

    // Print pressures on display
    printData(indoor_data.pres, outdoor_data.pres, pres);
    delay(4000);
    lcd.clear();

    // Print humidities on display
    printData(indoor_data.hum, outdoor_data.hum, hum);
    delay(4000);
    lcd.clear();
}


RadioPacket receiveData(){

    RadioPacket _radioData = {-99.0, -99.0, -99.0};

    if (_radio.hasData())
    {
        _radio.readData(&_radioData);
    }

    return _radioData;
}


void printData(float indoor_val, float outdoor_val, TypeOfVal type_of_val){

    if (type_of_val == temp)
    {
        lcd.setCursor(4,0);
        lcd.print(indoor_val);
        lcd.setCursor(10,0);
        lcd.print((char)223);
        lcd.print("C");
        lcd.setCursor(4,1);
        lcd.print(outdoor_val);
        lcd.setCursor(10,1);
        lcd.print((char)223);
        lcd.print("C");
    }

    else if (type_of_val == pres)
    {
        lcd.setCursor(3,0);
        lcd.print(indoor_val);
        lcd.setCursor(9,0);
        lcd.print(" hPa");
        lcd.setCursor(3,1);
        lcd.print(outdoor_val);
        lcd.setCursor(9,1);
        lcd.print(" hPa");
    }

    else if (type_of_val == hum)
    {
        lcd.setCursor(5,0);
        lcd.print(indoor_val);
        lcd.setCursor(10,0);
        lcd.print("%");
        lcd.setCursor(5,1);
        lcd.print(outdoor_val);
        lcd.setCursor(10,1);
        lcd.print("%");
    }

    else
    {
        lcd.setCursor(0,1);
        lcd.print("Unknown data type!");
    }
}