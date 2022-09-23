#ifdef OLED

int DisplayOK = 0;
int DisplayCleared = 0;

void SetupOLED()
{
  char Temp[32];
  
  // Wire.begin(OLED_SDA, OLED_SCL);

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false))
  {
    DisplayOK = 1;
    Serial.println("OLED OK");
    
    display.clearDisplay();  
    display.setTextColor(WHITE, 0);
    display.setTextSize(1);

    ShowSettings();
  }
  else
  {
    Serial.println("OLED Fail");
  }
}

void ShowGPSStatus(void)
{
  char Temp[32];

  if (DisplayOK)
  {
    if (GPS.Altitude > 2000)
    {
      if (!DisplayCleared)
      {
        display.clearDisplay();  
        DisplayCleared = 1;
      }
    }
    else
    {
      sprintf(Temp, "%02d:%02d:%02d %ldm %dsats", GPS.Hours, GPS.Minutes, GPS.Seconds, GPS.Altitude, GPS.Satellites);
      DisplayLine(3, Temp);

      sprintf(Temp, "%.5lf, %.5lf", GPS.Latitude, GPS.Longitude);
      DisplayLine(4, Temp);
    }
  }
}

void ShowSettings(void)
{
  char Temp[32], Temp2[8];
  
  sprintf(Temp, "Payload ID %s", Settings.PayloadID);
  DisplayLine(0, Temp);
 
  sprintf(Temp, "%.3lf MHz Mode %d", Settings.LoRaFrequency, Settings.LoRaMode);
  DisplayLine(1, Temp);

  if (Settings.LoRaCycleTime > 0)
  {
    sprintf(Temp, "TDM Slot %d/%d", Settings.LoRaSlot, Settings.LoRaCycleTime);
  }
  else
  {
    strcpy(Temp, "Continuous Tx");
  }

  if (Settings.CallingCount > 0)
  {
    sprintf(Temp2, " Call %d", Settings.CallingCount);
    strcat(Temp, Temp2);
  }
  DisplayLine(2, Temp);
}

void ShowTxStatus(char *RadioType, unsigned int Counter)
{
  char Temp[32];

  if (DisplayOK)
  {
    if (!DisplayCleared)
    {
      if (Counter > 0)
      {
        sprintf(Temp, "%02d:%02d:%02d %s:%d", GPS.Hours, GPS.Minutes, GPS.Seconds, RadioType, Counter);
      }
      else
      {
        sprintf(Temp, "%02d:%02d:%02d %s", GPS.Hours, GPS.Minutes, GPS.Seconds, RadioType);
      }
      DisplayLine(5, Temp);
    }
  }
}

void DisplayLine(int Line, char *Message)
{
  #define ROW_HEIGHT 10
  int y;
  
  y = Line * ROW_HEIGHT;
  
  display.fillRect(0, y, 128, ROW_HEIGHT, 0);

  display.setCursor(0,y);
  display.print(Message);
  display.display();
}

#endif
