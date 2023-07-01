

void FeatureSet_Indicator2 () {

#define X_Offset 170
#define Y_Offset 20

  int featureDelay = 100;
  delay(featureDelay);

#ifdef splashScreenLS
  tft.setRotation(1);// Rotate the display at the start:  0, 1, 2 or 3 = (0, 90, 180 or 270 degrees)
#endif

  tft.setFont(&Org_01);
  tft.setTextSize(1);


  //tft.drawBitmap(20 + X_Offset, 24 + Y_Offset, PWM_BL_BMP, 36, 36, ILI9341_GREY);
  //tft.setTextColor(ILI9341_GREY); tft.setCursor(30 + X_Offset, 68 + Y_Offset); tft.print("Off");

  // Fixed PWM PWM Brightness , Show default TFT backlight brightness setting

  tft.drawBitmap(82 + X_Offset, 5 + Y_Offset, PWM_BL_BMP, 36, 36, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE); tft.setCursor(92 + X_Offset, 60 + Y_Offset); tft.print(brightness_count);
  tft.fillCircle(100 + X_Offset, 23 + Y_Offset,  11,         ILI9341_YELLOW);
  tft.setTextColor(ILI9341_WHITE); tft.setCursor(85 + X_Offset, 50 + Y_Offset); tft.print("FIXED");

  delay(featureDelay);

  //------ 2A Tacho
#ifdef enable_NeopixelGauges
  tft.drawBitmap(82 + X_Offset, 85 + Y_Offset, Neo_Gauges_BMP, 36, 36, ILI9341_WHITE);

  // Show default NeoPixel brightness setting
  tft.setTextColor(ILI9341_WHITE); tft.setCursor(95 + X_Offset, 130 + Y_Offset); tft.print("NEO");
  //tft.setTextColor(ILI9341_WHITE); tft.setCursor(74 + X_Offset, 78 + Y_Offset); tft.print(NeoBrightness);

#else
  tft.drawBitmap(82 + X_Offset, 85 + Y_Offset, Neo_Gauges_BMP, 36, 36, ILI9341_GREY);

  tft.setTextColor(ILI9341_GREY); tft.setCursor(95 + X_Offset, 130 + Y_Offset); tft.print("Off");
#endif
  delay(featureDelay);



  //------ 5 BT
#ifdef enable_BT // Reserved
  tft.drawBitmap(82 + X_Offset, 150 + Y_Offset, BT_BMP, 36, 36, ILI9341_BLUE);
  tft.setTextColor(ILI9341_WHITE); tft.setCursor(95 + X_Offset, 195 + Y_Offset); tft.print("BT");

#else
  tft.drawBitmap(82 + X_Offset, 150 + Y_Offset, BT_BMP, 36, 36, ILI9341_GREY);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE); tft.setCursor(95 + X_Offset, 180 + Y_Offset); tft.print("USB");
  tft.setTextSize(1);

  tft.setTextColor(ILI9341_GREY); tft.setCursor(95 + X_Offset, 195 + Y_Offset); tft.print("OFF");
#endif


  //------ 5 BT and USB serial
#ifdef enable_DualSerialEvent
  tft.drawBitmap(188 + X_Offset, 24 + Y_Offset, BT_BMP, 36, 36, ILI9341_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN); tft.setCursor(190 + X_Offset, 45 + Y_Offset); tft.print("USB");
  tft.setTextSize(1);

  tft.setTextColor(ILI9341_WHITE); tft.setCursor(196 + X_Offset, 68 + Y_Offset); tft.print("DUAL");
#else
  //
#endif

  delay(2000);
  tft.setFont(); // Set back to default Adafruit GRFX font
}
