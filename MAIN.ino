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
// If value read from sensor is out of limit, alarm will be triggered.
float sensorLimits[4][2] = { { 1, 45 }, { 1, 60 }, { 1, 60 }, { -0.5, 0.5 } };

// Alarm Status
bool* sensorAlarms;
bool alarm;
bool alarmTemp;
bool alarmQuiet = true;

// Alarm blink states
bool alarmOutBlinkState = false;
bool alarmTextBlinkState = false;

// Button press time, stored to measure for how long button was pressed;
unsigned long buttonPressTime;
// Display state, 0 -> disabled 1 -> dimmed 2 -> full bdrigthess
int displayBrightness = 1;

extern unsigned int __bss_end;
extern void *__brkval;

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
  // Pin change interrupt configuration
  // Enable PCIE2 Bit3 = 1 (Port D)
  PCICR |= B00000100;
  //  Bit4 = 1 (Pin PD4)
  PCMSK2 |= B00010000;

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
  // Sensors
  float* sensorReadings = getSensorReadings();
  // Alarms
  setAlarms(sensorReadings);
  // Display
  setDisplay(sensorReadings);
  // Serial Port
  checkSerialPort(sensorReadings);
}

////////// SENSORS ////////////
float* getSensorReadings() {
  tempSensors.requestTemperatures();
  static float array[4];
  for (uint8_t i = 0; i < 3; ++i) {
    array[i] = tempSensors.getTempCByIndex(i);
  }
  array[3] = min(max(analogRead(Port_Pressure) * ((5.0 / 1023.0) * 0.767) - 1.38, -1), 2);
  return array;
}
////////// BUTTON ////////////
// Pin change interrupt
ISR(PCINT2_vect) {
  if (digitalRead(Port_Button) == LOW) {
    onButtonPress();
  } else {
    onButtonRelease();
  }
}

void onButtonPress() {
  buttonPressTime = millis();
}

void onButtonRelease() {
  unsigned long buttonPressedPeriod = millis() - buttonPressTime;
  if (buttonPressTime && buttonPressedPeriod > 20 && buttonPressedPeriod < 2000) {
    displayBrightness == 2 ? displayBrightness = 0 : displayBrightness++;
  }
  buttonPressTime = 0;
}

////////// DISPLAY ////////////

void setDisplay(float sensorReadings[]) {
  display.clearDisplay();
  if (displayBrightness || alarm) {
    display.dim(!(displayBrightness - 1));
    if (alarm) alarmTextBlinkState = !alarmTextBlinkState;
    display.setTextSize(1);
    //Alarm Icon
    if (alarmQuiet) {
      display.drawLine(121, 1, 121, 8, WHITE);
      display.drawLine(121, 0, 127, 0, WHITE);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(122, 1);
      display.println("Q");
      display.setTextColor(WHITE);
    }
    // Temperature header
    if (alarmTemp && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
    display.setTextSize(1);
    display.setCursor(17, 33);
    display.print("TEMPERATURE ('C)");
    display.setTextColor(WHITE);
    // Air In Temperature
    if (sensorAlarms[0] && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
    display.setCursor(0, 46);
    display.print("AIR IN");
    display.setCursor(3, 57);
    display.print(sensorReadings[0]);
    display.setTextColor(WHITE);
    // Water Temperature
    if (sensorAlarms[1] && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
    display.setCursor(48, 46);
    display.print("WATER");
    display.setCursor(48, 57);
    display.print(sensorReadings[1]);
    display.setTextColor(WHITE);
    // Air Out Temperature
    if (sensorAlarms[2] && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
    display.setCursor(86, 46);
    display.print("AIR OUT");
    display.setCursor(92, 57);
    display.print(sensorReadings[2]);
    display.setTextColor(WHITE);
    // Pressure
    if (sensorAlarms[3] && alarmTextBlinkState) display.setTextColor(BLACK, WHITE);
    display.setCursor(22, 0);
    display.println("WATER PRESSURE");
    display.setCursor(5, 13);
    display.setTextSize(2);
    if (sensorReadings[3] > 0) {
      display.print(" ");
    }
    display.print(sensorReadings[3]);
    display.println(" BAR");
    display.setTextColor(WHITE);
  }
  display.display();
}

////////// ALARMS ////////////

void setAlarms(float sensorReadings[]) {
  bool alarms[4];
  for (uint8_t i = 0; i < 4; ++i) {
    alarms[i] = sensorReadings[i] < sensorLimits[i][0] || sensorReadings[i] > sensorLimits[i][1];
  }
  sensorAlarms = alarms;
  alarmTemp = alarms[0] || alarms[1] || alarms[2];
  alarm = alarmTemp || alarms[3];
  handleAlarmQuiet();
}

void handleAlarmQuiet() {
  //
  if (buttonPressTime && millis() - buttonPressTime > 2000) {
    buttonPressTime = 0;
    alarmQuiet = !alarmQuiet;
  }
}

void handleAlarmOutput() {
  if (alarm && !alarmQuiet) {
    alarmOutBlinkState = !alarmOutBlinkState;
    if (alarmOutBlinkState) {
      digitalWrite(Port_Alarm, HIGH);
    } else {
      digitalWrite(Port_Alarm, LOW);
    }
  } else {
    alarmOutBlinkState = false;
    if (!alarm) alarmTextBlinkState = false;
    digitalWrite(Port_Alarm, LOW);
  }
}

////////// SERIAL PORT ////////////

void checkSerialPort(float sensorReadings[]) {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    if (command == "get sensors\n") {
      for (uint8_t i = 0; i < 4; ++i) {
        Serial.print(sensorReadings[i]);
        Serial.print(";");
      }
      Serial.println();
    }
    // set display brightness
    if (command.startsWith("set display brightness")) {
      char state = command[command.length() - 2];
      String options = "012";
      if (options.indexOf(state) != -1) {
        displayBrightness = state - '0';
      }
    }
    // disable buzzer and led while alarm is active
    if (command.startsWith("set alarm quiet")) {
      char state = command[command.length() - 2];
      String options = "01";
      if (options.indexOf(state) != -1) {
        alarmQuiet = !!(state - '0');
      }
    }
    Serial.flush();
  }
}
