#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Fonts/Org_01.h>
#include <analogWrite.h>
#include <TML_ErriezRotaryFullStep.h>

#include "Configuration_Settings.h" 
#include "Z_Bitmaps.h"

#include "BluetoothSerial.h" 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MCUFRIEND_kbv.h"
#include <Adafruit_NeoPixel.h>
#include <Pangodream_18650_CL.h>

#ifdef  RETRO_MONO
#define ILI9341_YELLOW      0xFFFF //ILI9341_WHITE
#define ILI9341_WHITE       0xFFFF //ILI9341_WHITE
#define ILI9341_BLUE        0xA510 //ILI9341_SILVER 
#define ILI9341_GREEN       0x7BEF //ILI9341_GREY 
#define ILI9341_RED         0xC618 //ILI9341_LIGHT_GREY
//--
#define ILI9341_SILVER      0xA510 //ILI9341_SILVER
#define ILI9341_GREY        0x7BEF //ILI9341_GREY 
#define ILI9341_LIGHT_GREY  0xC618 //ILI9341_LIGHT_GREY
#endif

#ifdef  RETRO_AMBER
#define ILI9341_YELLOW      0xFFE0 //ILI9341_YELLOW
#define ILI9341_WHITE       0xFFE0 //ILI9341_YELLOW
#define ILI9341_BLUE        0xA508 //ILI9341_GOLD
#define ILI9341_GREEN       0xA508 //ILI9341_GOLD
#define ILI9341_RED         0xA508 //ILI9341_GOLD
//--
#define ILI9341_SILVER      0xFFE0 //ILI9341_YELLOW
#define ILI9341_GREY        0xA508 //ILI9341_GOLD
#define ILI9341_LIGHT_GREY  0xA508 //ILI9341_GOLD
#endif


#ifdef  RETRO_GREEN
#define ILI9341_YELLOW      0x07E0 //ILI9341_GREEN
#define ILI9341_WHITE       0x07E0 //ILI9341_GREEN
#define ILI9341_BLUE        0x7BE0 //ILI9341_OLIVE
#define ILI9341_GREEN       0x7BE0 //ILI9341_OLIVE
#define ILI9341_RED         0x7BE0 //ILI9341_OLIVE
//--
#define ILI9341_SILVER      0x07E0 //ILI9341_GREEN
#define ILI9341_GREY        0x7BE0 //ILI9341_OLIVE
#define ILI9341_LIGHT_GREY  0x7BE0 //ILI9341_OLIVE
#endif

#define TX_LEDPin 18
/* Pre-define Hex NeoPixel colours,  eg. pixels.setPixelColor(0, BLUE); https://htmlcolorcodes.com/color-names/ */
#define neo_BLUE       0x0000FF
#define neo_GREEN      0x008000
#define neo_RED        0xFF0000
#define neo_ORANGE     0xFFA500
#define neo_DARKORANGE 0xFF8C00
#define neo_YELLOW     0xFFFF00
#define neo_WHITE      0xFFFFFF
#define neo_BLACK      0x000000 // OFF

//-----------------------------  TFT Colours  ---------------------------------
#define ILI9341_TEST        0x6A4E
#define ILI9341_BLACK       0x0000
#define ILI9341_WHITE       0xFFFF
#define ILI9341_GREY        0x7BEF
#define ILI9341_LIGHT_GREY  0xC618
#define ILI9341_GREEN       0x07E0
#define ILI9341_LIME        0x87E0
#define ILI9341_BLUE        0x001F
#define ILI9341_RED         0xF800
#define ILI9341_AQUA        0x5D1C
#define ILI9341_YELLOW      0xFFE0
#define ILI9341_MAGENTA     0xF81F
#define ILI9341_CYAN        0x07FF
#define ILI9341_DARK_CYAN   0x03EF
#define ILI9341_ORANGE      0xFCA0
#define ILI9341_PINK        0xF97F
#define ILI9341_BROWN       0x8200
#define ILI9341_VIOLET      0x9199
#define ILI9341_SILVER      0xA510
#define ILI9341_GOLD        0xA508
#define ILI9341_NAVY        0x000F
#define ILI9341_MAROON      0x7800
#define ILI9341_PURPLE      0x780F
#define ILI9341_OLIVE       0x7BE0

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

#ifdef batteryMonitor
/* Battery Monitor Settings*/
#define ADC_PIN 35        //!< ADC pin used, default is GPIO34 - ADC1_6 Voltage divider (2* 100K)
#define CONV_FACTOR 1.8 //!< Convertion factor to translate analog units to volts
#define READS 20
Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);
#endif
#define NEOPIN      5
#define NUM_PIXELS  8 
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);
int mode_Button     = 34; 
int display_Button_counter = 0;
int TFT_backlight_PIN = 0;
int brightness_countLast = 0;   
int ASPECT = 0; 
int displayDraw = 0;
boolean activeConn = false;
long lastActiveConn = 0;
boolean bootMode = true;
String inputString = "";
boolean stringComplete = false;
BluetoothSerial SerialBT;   
MCUFRIEND_kbv tft;

void setup() {
  SerialBT.begin(device_BT); 
  Serial.begin(baudRate); 
  inputString.reserve(220); 
  pixels.begin();   
  pixels.setBrightness(NeoBrightness); // Atmel Global Brightness
  pixels.show(); // Turn off all Pixels
  pinMode(mode_Button, INPUT_PULLUP);
#ifdef fixedBacklight
  pinMode(TFT_backlight_PIN, OUTPUT); 
#else
  analogWriteResolution(TFT_backlight_PIN, 12);
#endif
#ifdef enableTX_LED
  pinMode(TX_LEDPin, OUTPUT); 
#endif
backlightOFF();
 pinMode(TX_LEDPin, OUTPUT);
 digitalWrite(TX_LEDPin, LOW);
  delay(200); 
  int ID = tft.readID();
  tft.begin(ID); 
  tft.setRotation(ASPECT);// Rotate the display :  0, 1, 2 or 3 = (0, 90, 180 or 270 degrees)
  tft.setTextWrap(false); // Stop  "Loads/Temps" wrapping and corrupting static characters
  tft.setTextColor(ILI9341_WHITE);
  tft.fillScreen(ILI9341_BLACK);
  splashScreen();
}

void loop()
 {
  #ifdef enableTX_LED
  digitalWrite(TX_LEDPin, HIGH);    // turn the LED off HIGH(OFF) LOW (ON)
  #endif
    if(activeConn)
    {
      allNeoPixelsGREEN();
      allNeoPixelsOff();
    }else
    {
      allNeoPixelsBLUE();
    }
    button_Modes();
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
void allNeoPixelsOff() {
  for ( int i = 0; i < NUM_PIXELS; i++ ) {
    pixels.setPixelColor(i, 0, 0, 0 );
  }
  pixels.show();
}

void allNeoPixelsRED() {
  for ( int i = 0; i < NUM_PIXELS; i++ ) {
    pixels.setPixelColor(i, 255, 0, 0 );
  }
  pixels.show();
}

void allNeoPixelsGREEN() {
  for ( int i = 0; i < NUM_PIXELS; i++ ) {
    pixels.setPixelColor(i, 0, 255, 0 );
  }
  pixels.show();
}

void allNeoPixelsBLUE() {
  for ( int i = 0; i < NUM_PIXELS; i++ ) {
    pixels.setPixelColor(i, 0, 0, 255 );
  }
  pixels.show();
}

void serialBTEvent() {
  while (SerialBT.available()) {

    char inChar = (char)SerialBT.read();
    Serial.print(inChar); // Debug Incoming Serial

    // add it to the inputString:
    inputString += inChar;
    // if the incoming character has '|' in it, set a flag so the main loop can do something about it:
    if (inChar == '|') {
      stringComplete = true;

      delay(Serial_eventDelay);   //delay screen event to stop screen data corruption

#ifdef enableTX_LED
      /* Serial Activity LED */
      digitalWrite(TX_LEDPin, LOW);   // turn the LED off HIGH(OFF) LOW (ON)
#endif

    }
  }
}
//----------------------------- ActivityChecker  -------------------------------
void activityChecker() {

  if (millis() - lastActiveConn > lastActiveDelay)
  {   
    activeConn = false;
  }
  else
  {
    activeConn = true;
  }
  if (!activeConn) {


    /* Set Default Adafruit GRFX Font*/
    tft.setFont();

    tft.fillScreen(ILI9341_BLACK);

    tft.setRotation(0);// Rotate the display at the start:  0, 1, 2 or 3 = (0, 90, 180 or 270 degrees)
    tft.drawRoundRect  (0, 0  , 240, 400, 20,    ILI9341_BLUE);
    tft.setTextColor(ILI9341_BLUE);
    //tft.drawBitmap(82, 80, WaitingDataBMP_BT, 76, 154, ILI9341_RED);
    tft.drawBitmap(82, 80, WaitingDataBMP_BT,  76, 190, ILI9341_BLUE);

    tft.setTextSize(2); tft.setCursor(40, 40); tft.println("NO BLUETOOTH!!!");


    delay(2000);

    /* Clear Screen, Turn Off Backlight & Neopixels when there is no activity, */

    //tft.invertDisplay(0);
    backlightOFF ();
    allNeoPixelsOff();
    tft.fillScreen(ILI9341_BLACK);

    displayDraw = 0;

  }

}
//----------------------------- TFT Backlight  -------------------------------
#ifdef fixedBacklight 
void backlightON () {
  digitalWrite(TFT_backlight_PIN, HIGH);

}
void backlightOFF () {

  digitalWrite(TFT_backlight_PIN, LOW);
}
#else

void backlightON () {
  analogWrite(TFT_backlight_PIN, brightness_count); // TFT turn on backlight

}

void backlightOFF () {
  analogWrite(TFT_backlight_PIN, 0);        // TFT turn off backlight,

}

#endif
//----------------------------- Splash Screens --------------------------------
void splashScreen() {
  /* Initial Boot Screen, */
  allNeoPixelsBLUE();
  tft.setRotation(0);// Rotate the display at the start:  0, 1, 2 or 3 = (0, 90, 180 or 270 degrees)
  tft.setFont(&Org_01);
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRoundRect  (0, 0  , 240, 400, 8,    ILI9341_RED);
  #ifdef splashScreenLS // Quick landscape hack job, also in FeatureSet
  tft.setRotation(0);// Rotate the display at the start:  0, 1, 2 or 3 = (0, 90, 180 or 270 degrees)
  #endif
  tft.setRotation(1);

  tft.drawBitmap(44, 20, HSM_BG_BMP,  142, 128, ILI9341_WHITE);
  tft.drawBitmap(44, 20, HSM_BG2_BMP, 142, 128, ILI9341_RED);
  tft.drawBitmap(44, 20, HSM_BMP,     142, 128, ILI9341_GREY);

  tft.setTextSize(3);
  tft.setCursor(86, 140);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("NAM");
  tft.setTextSize(3);
  tft.setCursor(78, 160);
  tft.println("CODER");

  tft.setTextSize(2);
  tft.setCursor(22, 190);
  tft.setTextColor(ILI9341_SILVER);
  tft.print("PC Hardware Monitor");

  tft.setTextSize(3);
  tft.setCursor(22, 219);
  tft.setTextColor(ILI9341_RED);
  tft.print("HOAI NAM ");


  #ifdef batteryMonitor
  // Battery Level Indicator on Boot Screenn
  //-------------------------------------------------------------
  if (BL.getBatteryVolts() <= 3.4 ) {

    //tft.drawBitmap(170, 10, BATTERY_BMP, 60, 20, ILI9341_RED);
        tft.drawBitmap(170, 10, BT_BMP, 36, 36, ILI9341_RED);

    tft.setTextSize(2);
    tft.setCursor(178, 23);
    tft.setTextColor(ILI9341_BLACK);
    tft.print(BL.getBatteryVolts()); tft.print("v");

  } else {
    tft.drawBitmap(170, 10, BT_BMP, 36, 36, ILI9341_GREEN);
    tft.setTextSize(2);
    tft.setCursor(178, 23);
    tft.setTextColor(ILI9341_BLACK);
    tft.print(BL.getBatteryVolts()); tft.print("v");
  }
  #endif
  #ifdef batteryMonitor
    // Show Battery Level Indicator on waiting for data screen
    if (BL.getBatteryVolts() <= 3.4 ) {
    // tft.drawBitmap(33 + 40, 280, BATTERY_BMP, 60, 20, ILI9341_RED);
              tft.drawBitmap(33+40, 280, BT_BMP, 36, 36, ILI9341_RED);


      tft.setCursor(46 + 40, 286 ); // (Left/Right, UP/Down)
      tft.setTextSize(1);
      tft.setTextColor(ILI9341_BLACK);
      tft.print(BL.getBatteryVolts()); tft.print("v");
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      tft.setCursor(100 + 40, 283 ); // (Left/Right, UP/Down)
      tft.print(BL.getBatteryChargeLevel());
      tft.print("% ");

    } else {

    // tft.drawBitmap(33 + 40, 280, BATTERY_BMP, 60, 20, ILI9341_GREEN);
          tft.drawBitmap(33+40, 280, BT_BMP, 36, 36, ILI9341_RED);

      tft.setCursor(46 + 40, 286 ); // (Left/Right, UP/Down)
      tft.setTextSize(1);
      tft.setTextColor(ILI9341_BLACK);
      tft.print(BL.getBatteryVolts()); tft.print("v");
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      tft.setCursor(100 + 40, 283 ); // (Left/Right, UP/Down)
      tft.print(BL.getBatteryChargeLevel());
      tft.print("% ");
    }
  #endif
}
