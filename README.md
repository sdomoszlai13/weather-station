# Weather Station

**This repository contains code for a simple weather station with an outdoor module. The indoor module displays the temperature, humidity and pressure measured by both the indoor and the outdoor module. Both modules are Arduino-based and use a combinded AHT20 + BMP280 sensor. The indoor module utilizes an LCD to display measured values.**

The weather station consists of a module that can be placed in your house and an outdoor module that you can place outside. The modules take measurements about once every 20 seconds. The outdoor module sends the measured values to the base station that displays both inside and outside temperatures, humidities and pressures. To save energy, the display on the outdoor module is only turned on for a few seconds when a button is pressed on the module. However, measured values are always sent to the base station. For reasons of readability, the measured values are displayed in indoor-outdoor pairs on the screen of the base station.

## Files:
* base_station.ino: running on the base station's Arduino
* outdoor_sensor.ino: running on the outdoor unit's Arduino

Both the base station and the outdoor unit are utilize an Arduino Nano.

For more information, check out my blog post about this project: [https://sdomoszlai13.github.io/2023/04/06/weather-station.html](https://sdomoszlai13.github.io/2023/04/06/weather-station.html).
