#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

SoftwareSerial esp(10, 11);
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

#define ONE_WIRE_BUS 9
#define TEMPERATURE_PRECISION 12

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

int numberOfDevices, temp, buzzer = 8;
const int PulseWire = A0;
int myBPM, Threshold = 550;
PulseSensorPlayground pulseSensor;
unsigned long previousMillis = 0;
const long interval = 5000;

const int ecgPin = A0; // Analog pin for ECG sensor output
const int loPlusPin = 10; // Digital pin for LO+
const int loMinusPin = 11; // Digital pin for LO-

void setup()
{
  lcd.begin(16, 2);   

  Serial.begin(9600);  //To initailize serial communication 
  esp.begin(115200);
  sensors.begin();

  numberOfDevices = sensors.getDeviceCount();
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  pulseSensor.begin();

// Set pin modes
  pinMode(buzzer, OUTPUT);
  pinMode(ecgPin, INPUT);
  pinMode(loPlusPin, INPUT);
  pinMode(loMinusPin, INPUT);
  digitalWrite(buzzer, HIGH);

  lcd.setCursor(0, 0);
  lcd.print("  IoT Patient");
  lcd.setCursor(0, 1);
  lcd.print(" Monitor System");
  lcd.print("ECG Sensor Initialized");
  delay(1500);

  digitalWrite(buzzer, LOW);
  lcd.clear();
}

void loop()
{
  myBPM = pulseSensor.getBeatsPerMinute();
  if (pulseSensor.sawStartOfBeat())
  {
    beep();
    lcd.setCursor(0, 1);
    lcd.print("HEART:");
    lcd.print(myBPM);
    lcd.setCursor(9, 1);
    lcd.print(" BPM");
    delay(20);
  }
  sensors.requestTemperatures();
  for (int i = 0; i < numberOfDevices; i++)
  {
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      temp = printTemperature(tempDeviceAddress);
      lcd.setCursor(0, 0);
      lcd.print("BODY:");
      lcd.print(temp);
      lcd.print(" *C");
    }
  }

// Read the ECG signal
    int ecgValue = analogRead(ecgPin);

    // Check for lead off
    bool loPlus = digitalRead(loPlusPin);
    bool loMinus = digitalRead(loMinusPin);

    if (loPlus == 1 || loMinus == 1) {
        // If lead is off, print a warning
        Serial.println("Warning: Lead Off Detected!");
    } else {
        // Otherwise, print the ECG value
        Serial.println(ecgValue);
    }

    // Delay to allow for better visualization
    delay(1);

  upload();
}

int printTemperature(DeviceAddress deviceAddress)
{
  int tempC = sensors.getTempC(deviceAddress);
  return tempC;
}

void beep()
{
  digitalWrite(buzzer, HIGH);
  delay(150);
  digitalWrite(buzzer, LOW);
}

void upload()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    esp.print('*');
    esp.print(myBPM);
    esp.print(temp);
    esp.println('#');
  }
}

