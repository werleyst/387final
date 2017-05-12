/*
  Arduino Wind Speed Meter Anemometer mph - Adafruit anemometer (product ID 1733).
  Modified code created March 2016 from original code created by Joe Burg 11th November 2014 at http://www.hackerscapes.com/ with help from Adafruit forum users shirad
*/

//Initialise LCD display

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int serial_in;

//Setup Variables

double x = 0;
double y = 0;
const int sensorPin = A1; //Defines the pin that the anemometer output is connected to
int sensorValue = 0; //Variable stores the value direct from the analog pin
float sensorVoltage = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeed = 0; // Wind speed in meters per second (m/s)

float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
int sensorDelay = 2000; //Delay between sensor readings, measured in milliseconds (ms)

//Anemometer Technical Variables
//The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.

float voltageMin = .17; // Mininum output voltage from anemometer in mV.
float windSpeedMin = 1; // Wind speed in meters/sec corresponding to minimum voltage

float voltageMax = 4.5; // Maximum output voltage from anemometer in mV.
float windSpeedMax = 30; // Wind speed in meters/sec corresponding to maximum voltage

void setup()
{

  //Setup LCD display with welcome screen

  lcd.begin(16, 2);
  lcd.print("Geeky Gadgets");
  lcd.setCursor(0, 1);
  lcd.print("Windspeed Sensor");
  delay(2500);
  lcd.clear();
  lcd.setCursor(0, 0);
  Serial.begin(9600);  //Start the serial connection

}

//Anemometer calculations

void loop()
{
    sensorValue = analogRead(sensorPin); //Get a value between 0 and 1023 from the analog pin connected to the anemometer
    sensorVoltage = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage
    if (sensorVoltage <= voltageMin){ windSpeed = 0; //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer. Check if voltage is below minimum value. If so, set wind speed to zero.
    }else { windSpeed = (((sensorVoltage - voltageMin)*windSpeedMax)/(voltageMax - voltageMin));} //For voltages above minimum value, use the linear relationship to calculate wind speed in MPH.

  //Print voltage and windspeed to serial

  Serial.print("Voltage: ");
  Serial.print(sensorVoltage);
  Serial.print("\t");
  Serial.print("Wind speed: ");
  Serial.println(windSpeed);

  //Display Wind Speed results to LCD with Max wind speed

  lcd.setCursor(0, 0);
  lcd.print("Wind Speed mph");
  lcd.setCursor(0, 1);
  lcd.print(windSpeed);
  lcd.setCursor(7, 1);


  delay(sensorDelay);
}
