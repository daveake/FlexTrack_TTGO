unsigned long NextADC=0;

void SetupADC(void)
{
  //
}

void CheckADC(void)
{
  if (millis() >= NextADC)
  {
    GPS.BatteryVoltage = (int)(axp.getBattVoltage());
    
    NextADC = millis() + 10000L;
  }
}
