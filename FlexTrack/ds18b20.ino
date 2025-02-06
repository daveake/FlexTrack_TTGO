
#ifdef WIREBUS
#define PARASITE_POWER  false
#define OW_PIN          4
#define PARASITE_POWER  false
#define COMMON_RES      (DSTherm::RES_9_BIT)

unsigned long CheckDS18B20=0;

static Placeholder<OneWireNg_CurrentPlatform> _ow;

/* returns false if not supported */
static bool printId(const OneWireNg::Id& id)
{
    const char *name = DSTherm::getFamilyName(id);

    Serial.print(id[0], HEX);
    for (size_t i = 1; i < sizeof(OneWireNg::Id); i++) {
        Serial.print(':');
        Serial.print(id[i], HEX);
    }
    if (name) {
        Serial.print(" -> ");
        Serial.print(name);
    }
    Serial.println();

    return (name != NULL);
}

void StoreTemperature(const DSTherm::Scratchpad& scrpd)
{
  long temp = scrpd.getTemp();

  GPS.InternalTemperature = (float)temp / 1000;
  
  Serial.printf("DS18B20: %.1f C\n", GPS.InternalTemperature);
}

void Setupds18b20(void)
{
  new (&_ow) OneWireNg_CurrentPlatform(WIREBUS, false);

  DSTherm drv(_ow);

  drv.writeScratchpadAll(0, 0, DSTherm::RES_9_BIT);

  drv.copyScratchpadAll(PARASITE_POWER); 
}

void Checkds18b20(void)
{
  if (millis() >= CheckDS18B20)
  {
    DSTherm drv(_ow);
    Placeholder<DSTherm::Scratchpad> _scrpd;

    /* convert temperature on all sensors connected... */
    drv.convertTempAll(DSTherm::SCAN_BUS, PARASITE_POWER);

    /* ...and read them one-by-one */
    for (const auto& id: (OneWireNg&)_ow)
    {
        // if (printId(id)) {
      if (drv.readScratchpad(id, &_scrpd) == OneWireNg::EC_SUCCESS)
      {
        StoreTemperature(_scrpd);
      }
    }

    CheckDS18B20 = millis() + 10000L;
  }
}

#endif  
