

/*New Mode Button Function*/

void button_Modes() {

  int enc_buttonVal = digitalRead(mode_Button);
  if (enc_buttonVal == LOW)

  {
    delay(debounceButton); // Debounce Button
    display_Button_counter ++;

    /* Clear Screen*/
    backlightOFF();
    tft.fillScreen(ILI9341_BLACK);

    /* Reset count if over max mode number, */
    if (display_Button_counter == 5) // Number of screens available when button pressed
    {
          tft.fillScreen(ILI9341_BLACK);

      display_Button_counter = 0;
    }
  }

  else

    /* Change Mode */
    switch (display_Button_counter) {

      case 0: // 1st SCREEN
        ASPECT=0;
        Display_Port_Batt();

        break;

      case 1: // 2nd SCREEN
        // Display_LS_Batt();
        ASPECT=2;
        Display_Port_Batt();

        break;

      case 2: // 3nd SCREEN
        // Display_LS_Batt_180();
        ASPECT=1;
        Display_LS_Batt();

        break;

      case 3: // 4nd SCREEN
        // Display_Port_Batt_180();
        ASPECT=3;
        Display_LS_Batt();


        break;

      case 4: // 5th SCREEN
        Display_CircleGauge_Batt();
       break;

    }
}
