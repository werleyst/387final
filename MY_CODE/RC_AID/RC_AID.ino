#include "string.h"
#include "ctype.h"
#include "SoftwareSerial.h"
#include "dGPS.h"
//Code added to set up LCD screen. Add library
#include <LiquidCrystal.h>

// Software serial TX & RX Pins for the GPS module
// Initiate the software serial connection

int button = 7;
float desLat=0;                   //Destination Latitude filled by user in Serial Monitor Box
float desLon=0;                   //Destination Longitude filled by user in Serial Monitor Box
char fla[2];                      //flag (Y/N) whether to print checksum or not. Filled by user in Serial Monitor Box
char fla2[2];                     //flag (Y/N) whether to print Altitude, number of satellites used and HDOP. Filled by user in Serial Monitor Box
dGPS dgps = dGPS();               // Construct dGPS class

int displayType = 1; //used for different modes
int seconds = 00;
int minutes = 5;
bool start = false;


//Setup Variables for wind sensor _____________________________________________________________________

int serial_in;
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




//Code added to set up LCD screen. Pins needed
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  pinMode(button, INPUT);
  Serial.end();                  // Close any previously established connections
  Serial.begin(9600);            // Serial output back to computer.  On.
  dgps.init();                   // Run initialization routine for dGPS.
  
  //Code added to set up LCD screen. Screen Size
  lcd.begin(16, 2);
  delay(1000);   
}

void loop() {

  //used for button presses to change mode and start timer
  if (digitalRead(button) == LOW){
    if(displayType == 4 & !start){ 
      start = true;
      seconds=0;
      minutes=5;
    }else if(displayType == 4){
      lcd.clear();
      start = false;
      displayType = 1;
    }else
      lcd.clear();
      displayType++;
  }

  
  // gps coordinate mode
  if(displayType == 1){
    dgps.update(desLat, desLon);   
    lcd.setCursor(0, 0); // sets cursor on the first line
    lcd.print("Lat: ");
    lcd.print(dgps.Lat(), 6); //outputs gps latitude measurement to LCD screen
    
    lcd.setCursor(0, 1); // sets cursor on next line
    lcd.print("Lon: "); 
    lcd.print(dgps.Lon(), 6); //outputs gps longitude measurment to LCD screen
    
  } 

  //gps heading mode
  else if (displayType == 2){
      dgps.update(desLat, desLon);
      lcd.setCursor(0, 0); // sets cursor on the first line
      lcd.print("Heading: ");
      lcd.print(dgps.Head(), 6); //outputs gps latitude measurement to LCD screen
  }

  //wind sensor mode
  else if (displayType == 3){
      sensorValue = analogRead(sensorPin); //Get a value between 0 and 1023 from the analog pin connected to the anemometer
      sensorVoltage = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage
      if (sensorVoltage <= voltageMin){ windSpeed = 0; //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer. Check if voltage is below minimum value. If so, set wind speed to zero.
      }else { windSpeed = (((sensorVoltage - voltageMin)*windSpeedMax)/(voltageMax - voltageMin));} //For voltages above minimum value, use the linear relationship to calculate wind speed in MPH.

    lcd.setCursor(0, 0);
    lcd.print("Wind Speed mph");
    lcd.setCursor(0, 1);
    lcd.print(windSpeed);
    
  }

  //Countdown timer mode
  else if(displayType == 4){
    if(!(minutes == 0 & seconds==0)){
      lcd.setCursor(0, 0);
      lcd.print("Timer: ");
      lcd.print(minutes);
      lcd.print(":");
      if(seconds < 10)
        lcd.print("0");
      lcd.print(seconds);
    
      if(seconds ==0 & start == true){
        seconds = 60;
        minutes--;
      }
    }
  
    if(start == true)
      seconds--;
  }

  delay(1000);
}
