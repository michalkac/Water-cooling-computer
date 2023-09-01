Water cooling computer
This is a simple Arduino PC water cooling loop monitor i build for my new pc. 
Main motivation of this project was to monitor the pressure of my cooling loop, because it does not have any reservoir, so chances for leak caused by pressure changes are much higher.
After succesfully building a circuit that just monitored pressure and presented it on oled screen, considering how fast and easy it was, 
i decided to expand this project a bit, now it offers the following functionalities:
- Pressure sensor
- 3 temperature sensors
- OLED display that presents data from all sensors
- alarm activated if any of sensor values goes beyond given range, all 4 sensor values have independent screen blinking function that indicates which value is out of range.
- (To be Added) button that switch between display being turned off and two brightness levels (short press) and can activate/deactivate alarm (long press)
- (Work in progress) simple serial communication that allows to controll button functions and get sensor readings. You can use it for eample If You want to write an app to controll it/gather data/display graphs etc.
 

Parts used:

- Arduino Nano 
- SSD1306 128x64 OLED Display
- Pressure transducer (Output 0.5-4.5V -14.5-30psi, 5V  1/8" NPT, You can connect a diffrent one but the equation for getting pressure out of ADC reading might need to be adjusted to get proper pressure value.
- ADT-G14M-N18F Koolance adapter G1/4 BSPP male to 1/8" NPT female + a piece of teflon tape to connect pressure sensor to standard water cooling system thread
- 3x DS18B20 temperature sensor
- 5V buzzer with generator build in.
- red 3mm LED diode
- tactile button
- 20k 10k 4.7k resistors
- USB cable (preferably with internal usb 2.0 connector to connect it directly to motherbaord header)

Schematics:
