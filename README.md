# esp-homekit-multiple-sensors

This repository is based on the code and the examples provided by Maxim Kulkin.

https://github.com/maximkulkin/esp-homekit-demo

Detailed instructions on how to use the code found here can be found on Maxim's [wiki](https://github.com/maximkulkin/esp-homekit-demo/wiki/Build-instructions). 


In this repo there are two new examples: 

a) multiple_sensors

b) multiple_sensors_ac

The first example creates a multi-sensor device in Homekit. It includes a Temperature, Humidity, Light and Motion sensor. The second example, is like the first example but it also adds a thermostat accessory. With thermostat accessory you can control an A/C through IR commands. Any new features will be added to the second example. 

The following hardware can be used: 
![alt text](https://github.com/peros550/esp-homekit-multiple-sensors/blob/master/MultiSensor_AC_bb.jpg?raw=true)


You may also use the following link to create a ready made shield for wemos D1 mini. 
https://easyeda.com/peros550/wemos-d1-dht22-ir-pir-lm393_copy

Final project would look like this: 
![alt text](https://github.com/peros550/esp-homekit-multiple-sensors/blob/master/FinalProject1.jpg?raw=true)
![alt text](https://github.com/peros550/esp-homekit-multiple-sensors/blob/master/FinalProject2.jpg?raw=true)


Supported Features:
*Dynamic Wifi configuration by creating a soft AP

*OTA updates

*ThingSpeak integration to save temperature & humidity values

*Webserver to configure special features (expiremental)

# IR commands
If you are going to use IR, you need to supply your own set of RAW commands for your A/C in this [file](https://github.com/peros550/esp-homekit-multiple-sensors/blob/master/examples/multiple_sensors_ac/ac_commands.c). You may use Maxim's [esp-ir](https://github.com/maximkulkin/esp-ir) library to capture one by one each command for all modes (heating/cooling/auto/off). For AUTO mode there is only one command, I personally set there my A/C's FAN mode. 

# BOM
* 1 wemos d1 mini
* 1 DHT22 sensor
* 1 AM312 PIR sensor
* 1 LDR
* 3 Resistors (18 Ohm, 2.2K, 10K)
* 1 NPN transistor 2N2222
* 1 IR LED (Had good resutls with TSAL6200)
* 1 push button (is used for homekit reset) 

