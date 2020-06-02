unsigned long NextLEDs=0;

// LED modes:
// ON at startup
// 4Hz till GPS lock
// 1Hz with GPS lock
// OFF above 1000m (for power save)

void SetupLEDs(void)
{
  ControlLED(AXP20X_LED_LOW_LEVEL);
}

void ControlLED(axp_chgled_mode_t Mode)
{
  static axp_chgled_mode_t OldMode=AXP20X_LED_OFF;

  if (Mode != OldMode)
  {
    axp.setChgLEDMode(Mode);

    OldMode = Mode;
  }
}

void CheckLEDs(void)
{
  if (millis() >= NextLEDs)
  {
    if (GPS.Altitude > 1000)
    {
      ControlLED(AXP20X_LED_OFF);
    }
    else if (GPS.Satellites >= 4)
    {
      ControlLED(AXP20X_LED_BLINK_1HZ);
    }
    else
    {
      ControlLED(AXP20X_LED_BLINK_4HZ);
    }      
    
    NextLEDs = millis() + 1000L;
  }
}
