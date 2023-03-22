#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>
#include <NRFLite.h>


// Instantiate an AHT20 sensor
AHT20 aht20;

// Instantiate a BMP280 sensor
Adafruit_BMP280 bmp280;    // Use I2C interface
Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();

// Instantiate a display
LiquidCrystal_I2C lcd(0x27,20,4);    // Set address to 0x27 for a 16 X 2 display

// Instantiate a transceiver
NRFLite _radio;
const static uint8_t RADIO_ID = 0;    // This transceiver
const static uint8_t DESTINATION_RADIO_ID = 1;    // Other transceiver
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket    // Packet to be received
{
    uint8_t temp;
    uint32_t pres;
    uint32_t hum;
};


void setup(){

    Serial.begin(115200);
    Wire.begin();

    // Setup AHT20 sensor
    if (!aht20.begin()){

        Serial.println("Could not find a valid AHT20 sensor, check wiring"
                       " or try a different address!");
        while (1) delay(500);
    }

    Serial.println("AHT20 setup finished");

    // Setup BMP280 sensor
    if (!bmp280.begin()){

        Serial.println("Could not find a valid BMP280 sensor, check wiring"
                        " or try a different address!");
        while (1) delay(500);
    }

    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,   // Temperature oversampling
            Adafruit_BMP280::SAMPLING_X16,  // Pressure oversampling
            Adafruit_BMP280::FILTER_X16,    // Filtering
            Adafruit_BMP280::STANDBY_MS_500);   // Standby time
    Serial.println("BMP280 setup finished");
    
    // Setup display
    lcd.init();
    lcd.backlight();

    Serial.println("SSD1306 setup finished");

    // Setup transceiver
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){

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

    delay(50);

    // Print values on display
    RadioPacket indoor_data;
    indoor_data.temp = bmp280temp;
    indoor_data.pres = bmp280pres;
    printData(indoor_data);
    delay(3000);

    // Receive values
    RadioPacket outdoor_data = receiveData();

    // Print values on display
    printData(outdoor_data);
    delay(3000);
}


RadioPacket receiveData(){

    RadioPacket _radioData;

    if (_radio.hasData())
    {
        _radio.readData(&_radioData);
    }

    return _radioData;
}


void printData(RadioPacket weather_data){

    lcd.setCursor(1,0);
    lcd.print("Temp: ");
    lcd.setCursor(7,0);
    lcd.print(weather_data.temp);
    lcd.setCursor(1,1);
    lcd.print("Pres: ");
    lcd.setCursor(7,1);
    weather_data.pres = weather_data.pres / 100;
    lcd.print(weather_data.pres);
}