#ifdef CUTDOWN

unsigned long CutdownOffAt=0;

#ifdef SERVO_PIN
  Servo myservo;
#endif


void SetupCutdown(void)
{
  #ifdef CUTDOWN_PIN
    digitalWrite(CUTDOWN_PIN, 0);
    pinMode(CUTDOWN, OUTPUT);
    digitalWrite(CUTDOWN, 0);
  #endif

#ifdef SERVO_PIN
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(SERVO_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
  myservo.write(180);
#endif
}

void CutdownNow(unsigned long Period)
{
  Serial.println("CUTDOWN ON");

  #ifdef CUTDOWN_PIN
    digitalWrite(CUTDOWN, 1);
  #endif

#ifdef SERVO_PIN
  myservo.write(0);
#endif
  
  CutdownOffAt = millis() + Period;
}

void CutdownOff(void)
{
  Serial.println("CUTDOWN OFF");
  #ifdef CUTDOWN_PIN
    digitalWrite(CUTDOWN, 0);
  #endif

//#ifdef SERVO_PIN
//  myservo.write(180);
//#endif

  CutdownOffAt = 0;
}

void CheckCutdown(void)
{
  // Don't do anything unless we have GPS
  if (GPS.Satellites >= 4)
  {
    // Arm ?
    
    if ((GPS.Altitude > 2000) && (GPS.CutdownStatus == 0))
    {
      GPS.CutdownStatus = 1;      // Armed
    }

    // Trigger only if armed
    if (GPS.CutdownStatus == 1)
    {
      // ALTITUDE TEST
      if ((GPS.CutdownAltitude > 2000) && (GPS.Altitude > GPS.CutdownAltitude))
      {
        GPS.CutdownStatus = 2;      // Altitude trigger
        CutdownNow(GPS.CutdownPeriod);
      }
    }
  }

  if ((CutdownOffAt > 0) && (millis() >= CutdownOffAt))
  {
    CutdownOff();
  }
}

#endif
