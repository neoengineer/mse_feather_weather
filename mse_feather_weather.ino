/*----- Weather Station Display ---------------

  Copyright 2019, Micro Systems Engineering, LLC.
  Author - Thomas May (thomas.may@msepros.com)

  -----[ Program Description ]---------------

    Note - this code is based on the Quote Display for Adafruit ePaper FeatherWings
   * For use with Adafruit tricolor and monochrome ePaper FeatherWings
   * Written by Dan Cogliano for Adafruit Industries
   * Copyright (c) 2019 Adafruit Industries

  -----[ Hardware Configuration ]--------------- 
  Board - Adafruit ESP32 Feather
  Speed - 921600
  Flash Freq - 80MHz
  Display - Adafruit 2.13" eInk display featherwing #4195
   
  -----[ Software Configuration ]---------------                                              '
  Update the secrets.h file with your WiFi details
  Uncomment the ePaper display type below to match what you are using.
  Change the SLEEP setting to define the time between weather samples (1K / day are free)

  e-Ink display examples
  https://learn.adafruit.com/epaper-calendar-featuring-metro-m4-express-airlift-and-epaper-shield/arduino-code

  ESP Sleep
  https://github.com/espressif/esp-idf/blob/master/components/esp32/include/esp_sleep.h

  json documentation
  https://arduinojson.org
  https://arduinojson.org/v6/assistant/
  
  
  -----[ Revision History ]---------------
  V0.0 - Initial baseline.
  V0.1 - Added low battery detection with inverted display to allert the user 
  V0.2 - Fixed a bug where precipProb was looking at the hourly value, but hourly was not retrieved. Converted to
        using the minutely data for summary and precipProb. Since this requires a bigger json doc, switched to 
        a dynamic delaration for the doc object.
  
  
  This is still
          a slight issue. If we want the prob of rain over say the next 15 minutes, then we need to get the minutue
          forcast and average the prob for all the minutes over that time. However, there may be a memory limit since
          when trying to get teh hourly, there is json parse error
 

  
  -----[ Conventions ]---------------
  global const  All_CAPS
  global vars   all_lower_case
  Local Vars    all_lower_case
  Funct names  initialLowerCase

  Class names   CamelCase
  Class methods initialLowerCase
  Class prop    m_all_lower_case

  global const  All_CAPS
  global vars   all_lower_case
  Class names   InitialCaps
  Funct. names  initialLowerCase
  Class methods InitialCaps
  Class prop    m_initialLowerCase
  Local Vars    all_lower_case

  Feather  pin assignments
  
  D0 RX - 
  D1 TX - 
  D5 D5 - 
  D6 D6 - 
  D9/A7 - 
  D10 - 
  D11 
  D12 
  D13 LED
  
  D14/A0 
  D15/A1 
  D16/A2 
  D17/A3 
  D18/A4
  D19/A5 - Neopixel Data
  
  D20 SDA - 
  D21 SCL -
  
  D22 MISO - 
  D23 MOSI - 
  D24 SCK 
  
  GND - 
  3V3 - 
  VBat 
  VBus

**************************************************************************/



#include <Adafruit_GFX.h>    // Core graphics library https://learn.adafruit.com/adafruit-gfx-graphics-library/overview
#include <HTTPClient.h>
#include <ArduinoJson.h>     //https://github.com/bblanchon/ArduinoJson
#include <Adafruit_EPD.h>
#include <TimeLib.h>
#include "secrets.h"

// # define DEBUG
#include "DebugMacros.h"

// define the # of seconds to sleep before waking up and getting a new quote
#define SLEEP 900   // every 15 minutes

// WiFi timeout in seconds
#define WIFI_TIMEOUT 30

// Offset in hours from UTC time
#define UTC_OFFSET -5   // Standard Time
//#define UTC_OFFSET -4     // Daylight Savings Time

// What fonts do you want to use?
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h> // a font option for larger screens

// bigfont is the font for the big data display (left side)
const GFXfont *bigfont = &FreeSansBold12pt7b;

// smallfont is the font for the small data display (right side)
const GFXfont *smallfont = &FreeSansBold9pt7b;

// ofont is the font for the origin of the feed (bottom line)
const GFXfont *ofont = NULL;


// ESP32 settings
#define SD_CS       14
#define SRAM_CS     32
#define EPD_CS      15
#define EPD_DC      33 
#define LEDPIN      13
#define LEDPINON    HIGH
#define LEDPINOFF   LOW 

#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)

// Uncomment the following line if you are using 2.13" tricolor 212*104 EPD
//Adafruit_IL0373 epd(212, 104 ,EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
// Uncomment the following line if you are using 2.13" monochrome 250*122 EPD
Adafruit_SSD1675 epd(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// get string length in pixels
// set text font prior to calling this
int getStringLength(const char *str, int strlength = 0)
{
  char buff[1024];
  int16_t x, y;
  uint16_t w, h;
  if(strlength == 0)
  {
    strcpy(buff, str);
  }
  else
  {
    strncpy(buff, str, strlength);
    buff[strlength] = '\0';
  }
  epd.getTextBounds(buff, 0, 0, &x, &y, &w, &h);
  return(w);  
}

// word wrap routine
// first time send string to wrap
// 2nd and additional times: use empty string
// returns substring of wrapped text.
char *wrapWord(const char *str, int linesize)
{
  static char buff[255];
  int linestart = 0;
  static int lineend = 0;
  static int bufflen = 0;
  if(strlen(str) == 0)
  {
    // additional line from original string
    linestart = lineend + 1;
    lineend = bufflen;
    
    DPRINTLN("existing string to wrap, starting at position " + String(linestart) + ": " + String(&buff[linestart]));
  }
  else
  {
    DPRINTLN("new string to wrap: " + String(str));
    
    memset(buff,0,sizeof(buff));
    // new string to wrap
    linestart = 0;
    strcpy(buff,str);
    lineend = strlen(buff);
    bufflen = strlen(buff);
  }
  uint16_t w;
  int lastwordpos = linestart;
  int wordpos = linestart + 1;
  while(true)
  {
    while(buff[wordpos] == ' ' && wordpos < bufflen)
      wordpos++;
    while(buff[wordpos] != ' ' && wordpos < bufflen)
      wordpos++;
    if(wordpos < bufflen)
      buff[wordpos] = '\0';
    w = getStringLength(&buff[linestart]);
    if(wordpos < bufflen)
      buff[wordpos] = ' ';
    if(w > linesize)
    {
      buff[lastwordpos] = '\0';
      lineend = lastwordpos;
      return &buff[linestart];
    }
    else if(wordpos >= bufflen)
    {
      // first word too long or end of string, send it anyway
      buff[wordpos] = '\0';
      lineend = wordpos;
      return &buff[linestart];        
    }
    lastwordpos = wordpos;
    wordpos++;
  }
}

// return # of lines created from word wrap
int getLineCount(const char *str, int scrwidth)
{
  int linecount = 0;
  String line = wrapWord(str,scrwidth);

  while(line.length() > 0)
  {
    linecount++;
    line = wrapWord("",scrwidth);
  }
  return linecount;  
}

int getLineHeight(const GFXfont *font = NULL)
{
  int height;
  if(font == NULL)
  {
    height = 12;
  }
  else
  {
    height = (uint8_t)pgm_read_byte(&font->yAdvance);
  }
  return height;
}

// Retrieve page response from given URL
String getURLResponse(String url)
{
  HTTPClient http;
  String jsonstring = "";
  String err = "";

  DEBUG_PRINTLN( url );

  if(http.begin(url))
  {
    DPRINTLNF("[HTTP] GETting...");

    // start connection and send HTTP header
    int httpCode = http.GET();
    
    // httpCode will be negative on error
    if (httpCode > 0) {
      DPRINTLN("[HTTP] GET returned code: " + String(httpCode));
      
      // HTTP header has been sent and Server response header has been handled
      // file found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        jsonstring = http.getString();
      
        DEBUG_PRINTLN(jsonstring);
      }
    }
        
    // http get error
    else {
      err = "err: http get";
      DPRINTLN("[HTTP] GET... failed, error: " + http.errorToString(httpCode));
    }    

    http.end();
  }
  
  else {
    err = "err: http connect";
    DPRINTLNF("[HTTP] Unable to connect");
  }
  
  return jsonstring;
}


void getWeather(time_t &asoftime,
                String &summary, 
                float &prob, 
                float &temperature, 
                float &dewPoint, 
                float &humidity, 
                float &pressure, 
                float &windSpeed, 
                float &windBearing, 
                float &visibility)  
{
  // Arduino JSON docs - https://arduinojson.org/book/deserialization_tutorial6.pdf#page=10
  
  // StaticJsonDocument<1024> doc;
  DynamicJsonDocument doc(10000);
  
  // Set the url, api key and get the weather data
  String url = "https://api.darksky.net/forecast/84bc2d291eb87c91e2781fce14d0d53f/39.370788,-77.175910?exclude=hourly,daily,alerts,flags";
  String jsonweather = getURLResponse(url);

  DEBUG_PRINTLN(jsonweather.length());
  
  if(jsonweather.length() > 0)
  {
    // remove start and end brackets, jsonBuffer is confused by them
    // jsonweather = jsonweather.substring(1,jsonweather.length()-1);
    
    DEBUG_PRINT(jsonweather);

    DeserializationError error = deserializeJson(doc, jsonweather);
    if (error) 
    {   
      DPRINTLNF("json parseObject() failed - bad json");

      summary = "json parse error";
    }
    else
    {     
      time_t ttime = doc["currently"]["time"];
      asoftime = ttime;
      // asoftime = doc["currently"]["time"];
      
      // V0.2 changed to minutely summary
      String tsummary = doc["minutely"]["summary"];
      summary = tsummary;
      
      // Compute the average probability over 15 minutes
      prob = 0;
      for (int minute = 0; minute < 15; minute++) {  
        float tprobability = doc["minutely"]["data"][minute]["precipProbability"];
        prob += tprobability;
      }
      prob = prob / 15.0;
      
      float ttemperature = doc["currently"]["temperature"];
      temperature = ttemperature;
      
      float tdewPoint = doc["currently"]["dewPoint"];
      dewPoint = tdewPoint;
      
      float thumidity = doc["currently"]["humidity"];
      humidity = thumidity;
      
      float tpressure = doc["currently"]["pressure"];
      pressure = tpressure;
      
      float twindspeed = doc["currently"]["windSpeed"];
      windSpeed = twindspeed;

      float twindBearing = doc["currently"]["windBearing"];
      windBearing = twindBearing;
      
      float tvisibility = doc["currently"]["visibility"];
      visibility = tvisibility;
    }
  }
  else
  {
    summary = "err: retrieving URL";
  }
}


///void printTitle(char *Line1)
void printTitle(String &Line1)
{
  epd.setFont(smallfont);
  epd.setTextWrap(false);
  //int ypos = getLineHeight(smallfont) - 5;
  int ypos = 12;

  DPRINTLN("Title y postition = " + String(ypos) );

  epd.setCursor(0,ypos);
  epd.print(Line1);
}


void printBigData(String Line1, String Line2)
{
  // Adafruit SSD1675 Display is 250 x 122 - 0,0 is upper left
  // 
  epd.setFont(bigfont);
 
  epd.setCursor(20,50);
  epd.print(Line1);
  
  // Print a small o as a degree symbol
  epd.setCursor(epd.getCursorX() + 2, epd.getCursorY() - 10 );
  epd.setFont(smallfont);
  epd.print("o");
   
  epd.setFont(bigfont);
  epd.setCursor(20,80);
  epd.print(Line2);

  //int16_t x1, y1;
  //uint16_t w, h;
  //epd.getTextBounds(Line1, 20, 50, &x1, &y1, &w, &h);
  // epd.drawCircle(x1 + w + 7, y1, 5, 0x0000);
  // epd.setCursor(x1 + w + 2, y1 + 6);
}

void printSmallData(String Line1, String Line2, String Line3, String Line4, String Line5)
{
  // Adafruit SSD1675 Display is 250 x 122 - 0,0 is upper left
  epd.setFont(smallfont);
  
  int ypos = 30; // Location of first row
  int yadvance = getLineHeight(smallfont) - 3;
  
  epd.setCursor(100,ypos);
  epd.print(Line1);

  ypos += yadvance;
  epd.setCursor(100,ypos);
  epd.print(Line2);
  
  ypos += yadvance;
  epd.setCursor(100,ypos);
  epd.print(Line3);

  ypos += yadvance;
  epd.setCursor(100,ypos);
  epd.print(Line4);
  
  ypos += yadvance;
  epd.setCursor(100,ypos);
  epd.print(Line5);  
}

void printOther(String other)
{
  // Adafruit SSD1675 Display is 250 x 122 - 0,0 is upper left
  
  epd.setFont(ofont);
  
  // Center the text on the screen
  int x = getStringLength(other.c_str());
  int xpos = epd.width() / 2 - x / 2;
  // int xpos = 2;
  
  // Set to botton line of display
  int ypos = epd.height();

  // if using system default font (5x8), origin is upper left
  if (ofont == NULL) {
    ypos = epd.height() - 8;
  }
  
  epd.setCursor(xpos,ypos);
  epd.print(other);
}


void printVoltage(float batteryVoltage)
{
  // Adafruit SSD1675 Display is 250 x 122 - 0,0 is upper left
  
  epd.setFont(ofont);
  
  // Left justify the text on the screen
  int xpos = 0;
  
  // Set to botton line of display
  int ypos = epd.height();

  // if using system default font (5x8), origin is upper left
  if (ofont == NULL) {
    ypos = epd.height() - 8;
  }
  
  epd.setCursor(xpos,ypos);
  epd.print(batteryVoltage, 2);
  epd.print("V");
}


void setup() {

  time_t  asoftime;
  String  summary; 
  float   prob, 
          temperature, 
          dewPoint, 
          humidity, 
          pressure, 
          windSpeed, 
          windBearing, 
          visibility;

  SERIAL_BEGIN_WAIT(115200);

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LEDPINON);

/*  
  analogReference(DEFAULT);

  while(1){
    float batt = (float)analogRead(A13);
    float batteryVoltage = batt / 4095.0 * 3.3 * 1.139 * 2.0;
    Serial.print(batt);
    Serial.print("  ");
    Serial.println(batteryVoltage,2);
  }
*/

  // Initialize the eInk display
  epd.begin();
  epd.clearBuffer();
  epd.setTextWrap(false);
  
  DPRINTLNF("ePaper display initialized");

  // Initialize the wifi and connect to the network specified in secrets.h
  DPRINTLNF("Connecting to WiFi ");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter < WIFI_TIMEOUT) {
    delay(1000);
    counter++;
    
    DPRINTF(".");
  }
          
  // If connected, then get weather data
  if(WiFi.status() == WL_CONNECTED)
  {    
    DPRINTLNF("connected");

    getWeather( asoftime,
                summary, 
                prob, 
                temperature, 
                dewPoint, 
                humidity, 
                pressure, 
                windSpeed, 
                windBearing, 
                visibility);  
  }
  
  else
  {
    summary = "err: WiFi timed out";
    
    DPRINTF("Failed to connect to WiFi: ");
    DEBUG_PRINT(WIFI_SSID);
    DEBUG_PRINTLN(WIFI_PASSWORD);
  }
  
  // Check the current battery voltage
  float analogBatteryReading = (float)analogRead(A13);
  
  // divide by 4095 to get % of max value
  // assume adc attenuation set to 11db, so max reading is 3.3V - 
  // measured voltage and calculated internal vref must be 1130mv rather than 1100mv...
  // times 2 since feather uses a resistor divider in vbatt to divide by 2
  // 2245 to 2249 = 2.072V
  float callibratedBatteryVoltage = analogBatteryReading / 4095.0 * 3.3 * 1.139 * 2.0;

  // If the battery voltage is low, then invert the display to allert the user...
  if (callibratedBatteryVoltage < 3.5)
  {
    epd.setTextColor(EPD_INVERSE);
    epd.fillScreen(EPD_BLACK);
  }
  else 
  {
    epd.setTextColor(EPD_BLACK);    
  }
  
  printVoltage(callibratedBatteryVoltage);
  
  // Create and print the title line
  
  asoftime = asoftime + (3600 * UTC_OFFSET);  // adjust the weather data timestamp for DST and timezone  
  char buffer [10];
  sprintf (buffer, "%02u:%02u - ", hour(asoftime), minute(asoftime));  
  String titleLine(buffer);                   // convert the formatted time to a String  
  titleLine += summary;                       // aapend the summary to the time
  printTitle(titleLine);


  // Create and print the Big Data area
  
  printBigData( String(temperature,1), String(prob*100,0) + "%" );
  pressure = pressure * 0.02953;              // convert milibars to inches Hg

  // Create and print the Small Data area
    
  printSmallData( "WS - " + String(windSpeed,1) + "@" + String(windBearing,0),
                  "Hum  - " + String(humidity * 100,0) + "%",
                  "BP  - " + String(pressure,2) +"\"Hg",
                  "DP  - " + String(dewPoint,1),
                  "Vis - " + String(visibility,1) + "mi" );

  // Create and print the attribution line                 
  printOther("Powered by Dark Sky");

  epd.display();
  
  DPRINTLNF("done, going to sleep...");
    
  // power down ePaper display
  epd.powerDown();
  
  // If the battery is low, display warning and adjust sleep to 
  // put microcontroller to sleep, wake up after specified time
  ESP.deepSleep(SLEEP * 1e6);
}

/*
https://forums.adafruit.com/viewtopic.php?f=57&t=126839&p=632550&hilit=esp32+6ma#p632550

void deep_sleep_start(void){
  mgos_gpio_write(led_pin,0);           //led off
  ESP_ERROR_CHECK(esp_wifi_stop()); 
  ESP_ERROR_CHECK(esp_bluedroid_disable());
  ESP_ERROR_CHECK(esp_bluedroid_deinit());
  ESP_ERROR_CHECK(esp_bt_controller_disable());
  ESP_ERROR_CHECK(esp_bt_controller_deinit());
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
  esp_deep_sleep_start(); //zzz...
}
*/


void loop() {
  // should never get here, setup() puts the CPU to sleep
}


/*
ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0018,len:4
load:0x3fff001c,len:928
ho 0 tail 12 room 4
load:0x40078000,len:8424
ho 0 tail 12 room 4
load:0x40080400,len:5868
entry 0x4008069c
ePaper display initialized
Connecting to WiFi .connected
getting url: https://api.darksky.net/forecast/84bc2d291eb87c91e2781fce14d0d53f/39.370788,-77.175910?exclude=minutely,hourly,daily,alerts,flags
[HTTP] GET...
[HTTP] GET... code: 200
{"latitude":39.370788,"longitude":-77.17591,"timezone":"America/New_York","currently":{"time":1567094862,"summary":"Clear","icon":"clear-day","nearestStormDistance":156,"nearestStormBearing":17,"precipIntensity":0,"precipProbability":0,"temperature":74.9,"apparentTemperature":74.9,"dewPoint":58.07,"humidity":0.56,"pressure":1016.51,"windSpeed":8.22,"windGust":8.22,"windBearing":310,"cloudCover":0,"uvIndex":8,"visibility":10,"ozone":304.1},"offset":-4}

{"latitude":39.370788,"longitude":-77.17591,"timezone":"America/New_York","currently":{"time":1567098091,"summary":"Clear","icon":"clear-day","nearestStormDistance":140,"nearestStormBearing":75,"precipIntensity":0,"precipProbability":0,"temperature":76,"apparentTemperature":76,"dewPoint":57.27,"humidity":0.52,"pressure":1016.01,"windSpeed":7.43,"windGust":7.43,"windBearing":295,"cloudCover":0.06,"uvIndex":8,"visibility":10,"ozone":302.3},"offset":-4}

[{"text":"Human history becomes more and more a race between education and catastrophe","author":"H. G. Wells"}]


*/


/*

void printQuote(String &quote)
{
  int x = 0;
  int y = 0;
  bool bsmallfont = false;
  epd.setTextColor(EPD_BLACK);
  epd.setFont(qfont);
  epd.setTextSize(1);

  int scrwidth = epd.width() - 8;
  Serial.println("Screen width is " + String(scrwidth));
  Serial.println("Screen height is " + String(epd.height()));
  int linecount = getLineCount(quote.c_str(),scrwidth);
  int lineheightquote = getLineHeight(qfont);
  int lineheightauthor = getLineHeight(afont);
  int lineheightother = getLineHeight(ofont);
  int maxlines = (epd.height() - (lineheightauthor + lineheightother)) / lineheightquote;
  Serial.println("maxlines is " + String(maxlines));
  Serial.println("line height is " +String(lineheightquote));
  Serial.println("linecount is " +String(linecount));
  int topmargin = 0;
  if(linecount > maxlines)
  {
    // too long for default font size
    // next attempt, reduce lineheight to .8 size
    lineheightquote = .8 * lineheightquote;
    maxlines = (epd.height() - (lineheightauthor + lineheightother)) / lineheightquote;
    if(linecount > maxlines)
    {
      // next attempt, use small font
      epd.setFont(smallfont);
      bsmallfont = true;
      epd.setTextSize(1);
      lineheightquote = getLineHeight(smallfont);
      maxlines = (epd.height() - (lineheightauthor + lineheightother)) / lineheightquote;
      linecount = getLineCount(quote.c_str(),scrwidth);
      if(linecount > maxlines)
      {
        // final attempt, last resort is to reduce the lineheight to make it fit
        lineheightquote = (epd.height() - (lineheightauthor + lineheightother)) / linecount;
      }
    }
    Serial.println("maxlines has changed to " + String(maxlines));
    Serial.println("line height has changed to " +String(lineheightquote));
    Serial.println("linecount has changed to " +String(linecount));
  }
  if(linecount <= maxlines)
  {

    topmargin = (epd.height() - (lineheightauthor + lineheightother) - linecount*lineheightquote)/2;
    if(!bsmallfont)
      topmargin+=lineheightquote-4;
    //Serial.println("topmargin = " + String(topmargin));
  }
  String line = wrapWord(quote.c_str(),scrwidth);

  int counter = 0;
  epd.setTextColor(EPD_BLACK);
  while(line.length() > 0)
  {
    counter++;
    Serial.println("printing line " + String(counter) + ": '" + line + String("'"));
    epd.setCursor(x +4, y + topmargin);
    epd.print(line);
    y += lineheightquote;
    line = wrapWord("",scrwidth);
  }
}

void printAuthor(String author)
{
  epd.setTextColor(EPD_BLACK);
  epd.setFont(afont);
  int lineheightauthor = getLineHeight(afont);
  int lineheightother = getLineHeight(ofont);
  int x = getStringLength(author.c_str());
  // draw line above author
  epd.drawLine(epd.width() - x - 10, epd.height() - (lineheightauthor + lineheightother) + 2, epd.width(), epd.height() - (lineheightauthor + lineheightother) + 2, EPD_RED);
  epd.drawLine(epd.width() - x - 10, epd.height() - (lineheightauthor + lineheightother) + 2, epd.width() - x - 10,epd.height() - lineheightother - lineheightauthor/3, EPD_RED);
  epd.drawLine(0, epd.height() - lineheightother - lineheightauthor/3, epd.width() - x - 10,epd.height() - lineheightother - lineheightauthor/3, EPD_RED);
  // draw author text
  int cursorx = epd.width() - x - 4;
  int cursory = epd.height() - lineheightother - 2;
  if(afont == NULL)
  {
    cursory = epd.height() - lineheightother - lineheightauthor - 2 ;
  }
  epd.setCursor(cursorx, cursory);
  epd.print(author);
}
*/
