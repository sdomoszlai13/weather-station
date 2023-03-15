#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>

void setup()
{
    pinMode(13, OUTPUT);
}

void loop()
{
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(50);
}