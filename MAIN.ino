#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>

// Ports declarations
#define Port_Pressure A0
#define Port_OneWire PD2
#define Port_Alarm PD3
#define Port_Button PD4

// Sensor limits
//If value read from sensor is out of limit, alarm will be triggered.
float pressureMin = -1;
float pressureMax = 0.5;
float tempAirInMin = 1;
float tempAirInMax = 45;
float tempWaterMin = 1;
float tempWaterMax = 60;
float tempAirOutMin = 1;
float tempAirOutMax = 60;

// Alarm Status
bool alarm;
bool alarmTemp;
bool alarmPressure;
bool alarmTempAirIn;
bool alarmTempWater;
bool alarmTempAirOut;

// Alarm blink states
bool alarmOutBlinkState = false;
bool alarmTextBlinkState = false;

// Libs init
Adafruit_SSD1306 display(4);
OneWire oneWire(Port_OneWire);
DallasTemperature tempSensors(&oneWire);

void setup() {
  // Set up display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize with the I2C addr 0x3D if not working use 0x3C (for the 128x64)
  display.setTextColor(WHITE);
  display.dim(true);

  // Set up temperature sensors
  tempSensors.begin();

  // Set up button
  pinMode(Port_Button, INPUT);

  // Set up Alarm output and timer
  pinMode(Port_Alarm, OUTPUT);
  Timer1.initialize(100000);
  Timer1.attachInterrupt(handleAlarmOutput);

  // Set up serial
  Serial.begin(9600);
}


void loop() {
  // Get readings from all sensors
  
  // Returns pressure in BAR (0 ~= atmospheric pressurre). 
  // Equation sutiable for linear analog sensor 0.5-4.5V -14.5psi-30psi (from -1 to 2 BAR)
  float pressure = min(max(analogRead(Port_Pressure) * ((5.0 / 1023.0) * 0.767) - 1.38, -1), 2);

  tempSensors.requestTemperatures();
  float tempAirIn = tempSensors.getTempCByIndex(0);
  float tempWater = tempSensors.getTempCByIndex(1);
  float tempAirOut = tempSensors.getTempCByIndex(2);
  // Alarms
  setAlarms(pressure, tempAirIn, tempWater, tempAirOut);
  // Display
  handleDisplay(pressure, tempAirIn, tempWater, tempAirOut);
  // Serial Port
  handleSerialPort(pressure, tempAirIn, tempWater, tempAirOut);
}

void setAlarms(float pressure, float tempAirIn, float tempWater, float tempAirOut) {
  alarmPressure = pressure > pressureMax || pressure < pressureMin;
  alarmTempAirIn = tempAirIn > tempAirInMax || tempAirIn < tempAirInMin;
  alarmTempWater = tempWater > tempWaterMax || tempWater < tempWaterMin;
  alarmTempAirOut = tempAirOut > tempAirOutMax || tempAirOut < tempAirOutMin;
  alarmTemp = alarmTempAirIn || alarmTempWater || alarmTempAirOut;
  alarm = alarmTemp || alarmPressure;
}

void handleDisplay(float pressure, float tempAirIn, float tempWater, float tempAirOut) {
  display.clearDisplay();
  if (alarm) alarmTextBlinkState = !alarmTextBlinkState;
  // Pressure
  if (alarmPressure && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
  display.setCursor(22, 0);
  display.setTextSize(1);
  display.println("WATER PRESSURE");
  display.setCursor(5, 13);
  display.setTextSize(2);
  if (pressure > 0) {
    display.print(" ");
  }
  display.print(pressure);
  display.println(" BAR");
  display.setTextColor(WHITE);
  // Temperature header
  if (alarmTemp && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
  display.setTextSize(1);
  display.setCursor(17, 33);
  display.print("TEMPERATURE ('C)");
  display.setTextColor(WHITE);
  // Air In Temperature
  if (alarmTempAirIn && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 46);
  display.print("AIR IN");
  display.setCursor(3, 57);
  display.print(tempAirIn);
  display.setTextColor(WHITE);
  // Water Temperature
  if (alarmTempWater && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
  display.setCursor(48, 46);
  display.print("WATER");
  display.setCursor(48, 57);
  display.print(tempWater);
  display.setTextColor(WHITE);
  // Air Out Temperature
  if (alarmTempAirOut && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
  display.setCursor(86, 46);
  display.print("AIR OUT");
  display.setCursor(92, 57);
  display.print(tempAirOut);
  display.setTextColor(WHITE);
  display.display();
}

void handleSerialPort(float pressure, float tempAirIn, float tempWater, float tempAirOut) {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    if (command == "READ\n") Serial.println(pressure);
    if (command == "DIMM=true\n") display.dim(true);
    if (command == "DIMM=false\n") display.dim(false);
    Serial.flush();
  }
}

void handleAlarmOutput() {
  if (alarm == true) {
    alarmOutBlinkState = !alarmOutBlinkState;
    if (alarmOutBlinkState) {
      digitalWrite(Port_Alarm, HIGH);
    } else {
      digitalWrite(Port_Alarm, LOW);
    }
  } else {
    alarmOutBlinkState = false;
    alarmTextBlinkState = false;
    digitalWrite(Port_Alarm, LOW);
  }
}