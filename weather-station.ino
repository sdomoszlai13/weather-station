#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>


#define SCREEN_WIDTH 128 // OLED display width (in pixels)
#define SCREEN_HEIGHT 64 // OLED display height (in pixels)
#define OLED_RESET -1 // Reset pin number (-1: sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // Display address; 0x3D for 128 X 64, 0x3C for 128 X 32

AHT20 humidity_sensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
    Serial.begin(115200);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Loop forever
    }

    display.clearDisplay();    // Clear buffer
    Wire.begin();
    humidity_sensor.begin();
}

void loop()
{
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(50);
}


void printData(int humidity, float temp) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);    // Draw white text
    display.setCursor(0,0);    // Start at top left corner

    display.print("Relative humidity: ");
    display.print(humidity);
    display.print("%");
    display.print("\n");
    display.print("Temperature: ");
    display.print(temp, 1);

    display.display();
    delay(400);
}