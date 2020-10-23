#ifdef BMP085
unsigned long NextBMP=0;

Adafruit_BMP085 bmp;

void SetupBMP085(void)
{
  if (bmp.begin())
  {
    Serial.println("BMP085/180 found OK");
    
    NextBMP = millis() + 1;
  }
  else
  {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");}
}


void CheckBMP085(void)
{
  if ((NextBMP > 0) && (millis() >= NextBMP))
  {
    NextBMP = millis() + 1000;

    GPS.ExternalTemperature = bmp.readTemperature();
    GPS.Pressure = bmp.readPressure();
    
    Serial.printf("Ext Temp = %.1f C, Ext Pres = %.0f Pa\n", GPS.ExternalTemperature, GPS.Pressure);
  }
}

#endif
