#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>
#include <NRFLite.h>


// OUTDOOR SENSOR


volatile byte button_pressed = LOW;    // Global variable for the ISR to monitor button
const byte button_compare = HIGH;    // Byte to compare button_pressed to
const byte interrupt_pin = 2;    // Using digital pin 2 as interrupt pin


// Instantiate an AHT20 sensor
AHT20 aht20;

// Instantiate a BMP280 sensor
Adafruit_BMP280 bmp280;    // Use I2C interface
Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();

// Instantiate a display
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);    // Set display size

// Instantiate a transceiver
NRFLite _radio;
const static uint8_t RADIO_ID = 1;    // This transceiver
const static uint8_t DESTINATION_RADIO_ID = 0;    // Other transceiver
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket // Packet to be sent
{
    float temp;
    float pres;
    float hum;
};


void setup(){
    
    Serial.begin(115200);
    Wire.begin();

    // 
    pinMode(interrupt_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), buttonPressed, FALLING);

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

    Serial.println("BMP280 setup finished");

    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Normal mode
                       Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
                       Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                       Adafruit_BMP280::FILTER_X16,      // Filtering
                       Adafruit_BMP280::STANDBY_MS_500); // Standby time

    // Setup display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ // Address 0x3C for 128x32

        Serial.println(("SSD1306 allocation failed"));
        while (1) delay(500);
    }

    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(WHITE);
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

    float temp = bmp280temp;
    float pres = bmp280pres;
    float hum = aht20hum;

    // Print values on display if button is pressed
    if (button_pressed == button_compare){

        printData(temp, pres, hum);
        button_pressed = LOW;
        Serial.println("Button pressed, display activated");
    }

    delay(1000);

    // Send values to base station
    sendData(bmp280temp, bmp280pres, aht20hum);
}


void printData(float temp, float pres, float hum){

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    // Display temperature
    display.setCursor(0, 0);
    display.print(temp, 1);
    display.print(" C");
    display.display();
    delay(1000);
    display.clearDisplay();

    // Display pressure
    display.setCursor(0, 0);
    display.print(pres, 1);
    display.print(" hPa");
    display.display();
    delay(1000);
    display.clearDisplay();

    // Display humidity
    display.setCursor(0, 0);
    display.print(hum, 1);
    display.print("%");
    display.display();
    delay(1000);
    display.clearDisplay();

    display.print("");
    display.display();
}


void sendData(float temp, float pres, float hum){

    static RadioPacket _radioData;
    _radioData.temp = temp;
    _radioData.pres = pres;
    _radioData.hum = hum;

    Serial.println("Sending...");
    Serial.print(_radioData.temp);
    Serial.print(_radioData.pres);
    Serial.print(_radioData.hum);

    if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData)))
    {
        Serial.println("...Success");
    }
    else
    {
        Serial.println("...Failed");

    }
}


void buttonPressed(){

    button_pressed = HIGH;
}