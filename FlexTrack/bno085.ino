#ifdef BNO085
unsigned long NextBNO=0;

TwoWire I2CBMO = TwoWire(0);

Adafruit_BNO08x bno08x(0);    // RESET PIN
sh2_SensorValue_t sensorValue;

void setReports(void)
{
  Serial.println("Setting desired reports");
  /*
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  */
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    Serial.println("Could not enable geomagnetic rotation vector");
  }
  /*
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
  if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
    Serial.println("Could not enable step counter");
  }
  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
    Serial.println("Could not enable stability classifier");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
    Serial.println("Could not enable shake detector");
  }
  if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
    Serial.println("Could not enable personal activity classifier");
  }
  */
}

void SetupBNO085(void)
{
  I2CBMO.begin(21, 22, 100000);

  // Try to initialize!
  if (bno08x.begin_I2C(0x4B, &I2CBMO))
  {
    Serial.println("BNO08x Found!");
  
    for (int n = 0; n < bno08x.prodIds.numEntries; n++)
    {
      Serial.print("Part ");
      Serial.print(bno08x.prodIds.entry[n].swPartNumber);
      Serial.print(": Version :");
      Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
      Serial.print(".");
      Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
      Serial.print(".");
      Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
      Serial.print(" Build ");
      Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
    }

    setReports();
  
    NextBNO = millis() + 1;
  }
  else
  {
    Serial.println("Could not find a valid BNO085 sensor, check wiring and address!");
  }
}


void CheckBNO085(void)
{
  if ((NextBNO > 0) && (millis() >= NextBNO))
  {
    NextBNO = millis() + 100;

    if (bno08x.wasReset())
    {
      Serial.print("sensor was reset ");
      setReports();
    }
    
    if (!bno08x.getSensorEvent(&sensorValue))
    {
      return;
    }
    
    switch (sensorValue.sensorId)
    {
      case SH2_ACCELEROMETER:
        Serial.print("Accelerometer - x: ");
        Serial.print(sensorValue.un.accelerometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.accelerometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.accelerometer.z);
        break;
        
      case SH2_GYROSCOPE_CALIBRATED:
        Serial.print("Gyro - x: ");
        Serial.print(sensorValue.un.gyroscope.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.gyroscope.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.gyroscope.z);
        break;
        
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        Serial.print("Magnetic Field - x: ");
        Serial.print(sensorValue.un.magneticField.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.magneticField.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.magneticField.z);
        break;
        
      case SH2_LINEAR_ACCELERATION:
        Serial.print("Linear Acceration - x: ");
        Serial.print(sensorValue.un.linearAcceleration.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.linearAcceleration.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.linearAcceleration.z);
        break;
        
      case SH2_ROTATION_VECTOR:
        Serial.print("Rotation Vector - r: ");
        Serial.print(sensorValue.un.rotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.rotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.rotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.rotationVector.k);
        break;
        
      case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        Serial.printf("~GR%f,%f,%f,%f\n", sensorValue.un.geoMagRotationVector.real,
                                          sensorValue.un.geoMagRotationVector.i,
                                          sensorValue.un.geoMagRotationVector.j,
                                          sensorValue.un.geoMagRotationVector.k);
        GPS.Heading = (int)(sensorValue.un.geoMagRotationVector.i * 180);
        GPS.Pitch = (int)(sensorValue.un.geoMagRotationVector.j * 180);
        GPS.Roll = (int)(sensorValue.un.geoMagRotationVector.k * 180);
        break;
      
      case SH2_GAME_ROTATION_VECTOR:
        Serial.print("Game Rotation Vector - r: ");
        Serial.print(sensorValue.un.gameRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.gameRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.gameRotationVector.k);
        break;
      
      case SH2_STEP_COUNTER:
        Serial.print("Step Counter - steps: ");
        Serial.print(sensorValue.un.stepCounter.steps);
        Serial.print(" latency: ");
        Serial.println(sensorValue.un.stepCounter.latency);
        break;
          
      case SH2_RAW_ACCELEROMETER:
        Serial.print("Raw Accelerometer - x: ");
        Serial.print(sensorValue.un.rawAccelerometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawAccelerometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawAccelerometer.z);
        break;

      case SH2_RAW_GYROSCOPE:
        Serial.print("Raw Gyro - x: ");
        Serial.print(sensorValue.un.rawGyroscope.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawGyroscope.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawGyroscope.z);
        break;
        
      case SH2_RAW_MAGNETOMETER:
        Serial.print("Raw Magnetic Field - x: ");
        Serial.print(sensorValue.un.rawMagnetometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawMagnetometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawMagnetometer.z);
        break;
    }
  }
}

#endif
