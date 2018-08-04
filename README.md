# esp-homekit-multiple-sensors

This repository is based on the code and the examples provided by Maxim Kulkin.

https://github.com/maximkulkin/esp-homekit-demo

Detailed instructions on how to use the code found here can be found on Maxim's repository. It is the first thing to do before trying the code here. 

This multiple sensor example is basically Maxim Kulkin's temperature sensure example with the following improvements: 

A) Added Dynamic Wifi configuration by creating a soft AP so that there is no need to hard code wifi settings. 

B) Added a button to reset Wifi & Homekit configuration so that the Accessories could be reused

C) Added a motion sensor (with the help of @maccoylton and his repository: https://github.com/maccoylton/esp-homekit-motion-sensor ) 

D) Added a light sensor which provides an indication of brightness. (Home app reports the brightness level in the form of Lux units, but the actual measurements are not accurate at least in this version of code)  


For all these to work, I used a wemos D1 mini board, a motion sensor (HC-SR501 or AM312) , a Photodiode Light Sensor module,  a DHT22 , a breadboard and some jumber cables. 

Here are the pins used on the wemos: 

A0  Ligh sensor analog output
D1  DHT22 data pin
D2  Push button
D5  Motion Sensor

# Future work could include: 

-any bug fixes

-Implement OTA updates using (OTA) mechanism created by @HomeACcessoryKid

-Implement IR support (I would like to control my room's A/C)

