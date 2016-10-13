# SmartHome-18f2455_Shutters_DHT22

## Smart home controller for shutters and DHT22
Code for Microchip pic18f2550 microcontroller, developed in MplabX IDE using C18 compiler (not XC8 !).
The microcontroller communicates with the Smart Home server using XBee series 1 radios.
it serves two functions:
* Controll an electric shutter\blinds motor. this is done either via up\down switch or remotely via XBee transmission. 
* Manage a DHT22 temperature sensor and send the readings to the Smart Home server.

## Button Operation:
* Short press - strat moving up\down untill motion time (parameter) passes.
* Short press while in motion - Stop motion. 
* Long press - move untill switch is released


##Wiring diagram:
![My image](extras/wiring%20diagram.png)
