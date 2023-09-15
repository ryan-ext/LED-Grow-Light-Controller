/* This sketch implements PID control for LED grow lights capable
of accepting a 0 - 10 VDC signal. An Adafruit ADS1115 analog to digital
controller is converting the signal from a light sensor to serve as the
setpoint for the PID control. The signal from an Adafruit MCP4728
digital to analog converter is amplified to a maximum of 10 V with an LM358
opamp. A realtime clock is for to set light and darkness periods.

Written by Ryan Kurasaki


*/

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <SPI.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "RTClib.h"

RTC_PCF8523 rtc;
Adafruit_ADS1115 ads1115;
Adafruit_MCP4728 mcp;
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
double ppfd;
double v0;
double target = 150;
double Kp = 2, Ki = 5, Kd = 1;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

bool light_cycle = LOW;
int hour_on = 6;
int minute_on = 0;
int hour_off = 22;
int minute_off = 0;
int time_on;
int time_off;
int time_now;

float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

long start_time;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello!");
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  // initialize TFT
  tft.init(135, 240);  // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Setpoint = 250;
  delay(10);

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  ads1115.begin(0x48);
  ads1115.setGain(GAIN_SIXTEEN);  // 16x gain  +/- 0.256V  1 bit = 0.125mV
  Serial.println("Setup ADC complete");
  // Try to initialize!
  if (!mcp.begin(0x64)) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
    Serial.println("Setup DAC complete");
  }

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.start();

  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();


  Serial.println("Setup complete");
  start_time = millis();
}

void loop(void) {
  //Daylight Cycle - Convert into seconds
  DateTime now = rtc.now();
  time_on = hour_on * 3600 + minute_on * 60;
  time_off = hour_off * 3600 + minute_off * 60;
  time_now = now.hour() * 3600 + now.minute() * 60;

  if (time_now >= time_on && time_now <= time_off) light_cycle = HIGH;
  else light_cycle = LOW;



  int16_t results;
  float r = 0;
  int n = 1;
  start_time = millis();
  while (millis() - start_time < 1000) {
    results = ads1115.readADC_Differential_0_1();
    r = results + r;
    n = n + 1;
  }
  float r2;
  r2 = r / n;
  ppfd = r2 * 0.0078125 * 100;

  if (light_cycle == HIGH) {
    mcp.setChannelValue(MCP4728_CHANNEL_A, v0);

    error = Setpoint - ppfd;
    changeError = error - last_error;                                 // derivative term
    totalError += error;                                              //accumalate errors to find integral term
    pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);  //total gain
    pidTerm = constrain(pidTerm, 0, 4095);                            //constraining to appropriate value
    pidTerm_scaled = abs(pidTerm);                                    //make sure it's a positive value

    last_error = error;

    v0 = pidTerm;



  } else mcp.setChannelValue(MCP4728_CHANNEL_A, 0);

  mcp.setChannelValue(MCP4728_CHANNEL_A, v0);
  Serial.print("Differential: ");
  Serial.print(r2);
  Serial.print("(");
  Serial.print(r2 * 0.007812, 3);
  Serial.print("mV)");
  Serial.print("(");
  Serial.print(ppfd);
  Serial.print(" umol/m2/s)");
  Serial.print("(");
  Serial.print(v0 * 3.3 / 4095 * 3, 3);
  Serial.print(" V)");
  Serial.print("(");
  Serial.print(Setpoint);
  Serial.print(" umol/m2/s)");
  Serial.print(time_now);
  Serial.print('\t');
  Serial.print(time_on);
  Serial.print('\t');
  Serial.print(time_off);
  Serial.print('\t');
  Serial.println(light_cycle);


  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.print("PPFD   ");
  tft.println(ppfd);
  tft.print("Target ");
  tft.println(Setpoint);
  tft.print("Cycle  ");
  tft.println(light_cycle);
}