/*-------------------------------------------------------------------------------------------------------\
|                                                                                                        |
| FlexTrack specifically for the TTGO T-Beam which has ESP32, UBlox NEO-6M and LoRa transceiver          |
|                                                                                                        |
| Compared with usual AVR FlexTrack, it has landing prediction and EEPROM configuration.                 |
|                                                                                                        |
\*------------------------------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------------------------------

// CONFIGURATION SECTION.  CHANGE AS REQUIRED.

// #define LORA_DONT_LISTEN                  // Saves power

// Optional devices - uncomment if present
#define OLED                                // Enable OLED
#define WIREBUS 4
// #define BMP085         // Also works with BMP180
// #define BNO085

// Options
#define SERVO_PIN                15      // Cutdown via a servo motor
// #define CUTDOWN_PIN             25      // This pin made active to cut down from balloon
// #define ENABLE_UPLINK               // Enables uplink, for which you need to set TDM mode for transmission/reception

#if defined(SERVO_PIN) || defined(CUTDOWN_PIN)
  #define CUTDOWN
#endif

// PRODUCT INFO

#define   VERSION     "V1.22"
#define   PRODUCT     "FlexTrack"
#define   DESCRIPTION "TTGO T-Beam ESP32"

// FIXED CONFIG

#define SIG_1   'D'
#define SIG_2   'A'

#define LORA_TIME_INDEX      2
#define LORA_TIME_MUTLIPLER  2
#define LORA_TIME_OFFSET     1
#define LORA_PACKET_TIME    500
#define LORA_OFFSET           0         // Frequency to add in kHz to make Tx frequency accurate

#define LORA_CALL_FREQ     433.650
#define LORA_CALL_MODE     5        


// HARDWARE DEFINITION FOR TTGO T-BEAM

#define OLED_SDA                21
#define OLED_SCL                22
#define OLED_ADDRESS            0x3C
#define SCREEN_WIDTH            128
#define SCREEN_HEIGHT           64


// Libraries

#include <EEPROM.h>

#ifdef OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#ifdef BMP085
#include <Adafruit_BMP085.h>
#endif

#ifdef BNO085
#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_SensorValue.h>
#include <sh2_util.h>
#include <shtp.h>
#endif

#ifdef WIREBUS
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"
#endif

#ifdef SERVO_PIN
#include <ESP32Servo.h>
#endif

//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------

// HARDWARE DEFINITION

#define LORA_NSS           18                // Comment out to disable LoRa code
#define LORA_RESET         14                // Comment out if not connected
#define LORA_DIO0          26                
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI

//------------------------------te------------------------------------------------------------------------

// OLED
#ifdef OLED
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

// Cutdown ...
#ifdef CUTDOWN
  #define CUTDOWN_FIELD_FORMAT    ",%d"
  #define CUTDOWN_FIELD_LIST       ,GPS.CutdownStatus
#else
  #define CUTDOWN_FIELD_FORMAT    
  #define CUTDOWN_FIELD_LIST          
#endif

// IMU ...
#ifdef BNO085
  #define IMU_FIELD_FORMAT    ",%d,%d,%d"
  #define IMU_FIELD_LIST       ,GPS.Heading, GPS.Pitch, GPS.Roll

#else
  #define IMU_FIELD_FORMAT    
  #define IMU_FIELD_LIST          
#endif


#define SENTENCE_LENGTH       200        // Extend if needed
#define PAYLOAD_LENGTH         16
#define FIELDLIST_LENGTH       24
#define COMMAND_BUFFER_LENGTH  80

#include <axp20x.h>
AXP20X_Class axp;

//------------------------------------------------------------------------------------------------------
//
//  Globals

struct TSettings
{
  // Common
  char PayloadID[PAYLOAD_LENGTH];
  char FieldList[FIELDLIST_LENGTH];

  // GPS
  unsigned int FlightModeAltitude;
  
  // LoRa
  float LoRaFrequency;
//  unsigned char Implicit;             // 1=Implicit, 0=Explicit
//  unsigned char ErrorCoding;
//  unsigned char Bandwidth;
//  unsigned char SpreadingFactor;
//  unsigned char LowDataRateOptimize;
  int           LoRaMode;
  unsigned int  LoRaCycleTime;
  int           LoRaSlot;
  unsigned int  CallingCount;
  int           LoRaRepeatSlot1;
  int           LoRaRepeatSlot2;
  char          UseBinaryMode;
  char          BinaryNode;

  // Cutdown
  long          CutdownAltitude;
  unsigned int  CutdownPeriod;

  // Uplink
  int           EnableUplink;
  char          UplinkCode[16];

  // Prediction
  float         CDA;
  float         PayloadWeight;
  long          LandingAltitude;

  // RTTY
  float         RTTYFrequency;
  int           RTTYBaudRate;
  int           RTTYAudioShift;
  unsigned int  RTTYCount;
  unsigned int  RTTYEvery;
  unsigned int  RTTYPreamble;

  int           IncludeFieldList;
} Settings;

#define EEPROM_SIZE sizeof(Settings)

struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	BiSeconds;
	float		Latitude;
	float		Longitude;
	int32_t  	Altitude;
};  //  __attribute__ ((packed));

typedef enum {fmIdle, fmLaunched, fmDescending, fmLanding, fmLanded} TFlightMode;

struct TGPS
{
  int Hours, Minutes, Seconds;
  unsigned long SecondsInDay;					// Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude, MinimumAltitude, MaximumAltitude, PreviousAltitude;
  unsigned int Satellites;
  float Speed, Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  unsigned int BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  float AscentRate;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte GPSFlightMode;
  TFlightMode FlightMode;
  byte PowerMode;
  int CutdownStatus;
  float PredictedLatitude;
  float PredictedLongitude;
  float CDA;
  int UseHostPosition;
  int TimeTillLanding;
  float PredictedLandingSpeed;
  int LastPacketRSSI;
  int LastPacketSNR;
  int ReceivedCommandCount;
  char LastReceivedCommand[16];
  long CutdownAltitude;
  unsigned int CutdownPeriod;
  int Heading, Pitch, Roll;
} GPS;


unsigned int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------

void setup()
{
  // Serial port(s)

  setCpuFrequencyMhz(80);
  
  Serial.begin(38400);

  delay(1000);
  
  Serial.println();
  Serial.println();
  Serial.printf("%s: %s %s\n", PRODUCT, DESCRIPTION, VERSION); 
  Serial.println();

  EEPROM.begin(EEPROM_SIZE);  

  if ((EEPROM.read(0) == SIG_1) && (EEPROM.read(1) == SIG_2))
  {
    // Store current (default) settings
    Serial.println("Loading settings from EEPROM");
    LoadSettings();
  }
  else
  {
    Serial.println("Placing default settings in EEPROM");
    SetDefaults();
  }
  
  Serial.print("Payload ID ");
  Serial.println(Settings.PayloadID);
        
#ifdef CUTDOWN
  GPS.CutdownAltitude = Settings.CutdownAltitude;
  
  SetupCutdown();
#endif

  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
  {
    Serial.println("AXP192 OK");
  } else {
    Serial.println("AXP192 FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);   // LoRa On
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS On
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);  // ??
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);  // ??
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED Off
  // axp.setDCDC1Voltage(3300);
  
//  Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

  if (axp.isChargeing()) {
      Serial.println("Charging");
  }  

#ifdef OLED
  SetupOLED();
#endif

#ifdef BNO085
  SetupBNO085();
#endif

  SetupLEDs();

  SetupADC();
  
  SetupGPS();
  
  SetupLoRa();

#ifdef WIREBUS
  Setupds18b20();
#endif

#ifdef BMP085
  SetupBMP085();
#endif

  SetupPrediction();
}


void loop()
{  
  CheckGPS();

#ifdef CUTDOWN
  CheckCutdown();
#endif
  
  CheckLoRa();

  CheckADC();
   
  CheckLEDs();

#ifdef WIREBUS
  Checkds18b20();
#endif

#ifdef BMP085
  CheckBMP085();
#endif

#ifdef BNO085
  CheckBNO085();
#endif

  CheckHost();

  CheckPrediction();
}

void CheckHost(void)
{
  static char Line[COMMAND_BUFFER_LENGTH];
  static unsigned int Length=0;
  char Character;

  while (Serial.available())
  { 
    Character = Serial.read();

    if (Character == '~')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Character == '\r')
    {
      Line[Length] = '\0';
      ProcessCommand(Line+1);
      Length = 0;
    }
    else if (Length >= sizeof(Line))
    {
      Length = 0;
    }
    else if (Length > 0)
    {
      Line[Length++] = Character;
    }
  }
}

void ProcessCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'G')
  {
    OK = ProcessGPSCommand(Line+1);
  }
  else if (Line[0] == 'C')
  {
    OK = ProcessCommonCommand(Line+1);
  }
  else if (Line[0] == 'L')
  {
    OK = ProcessLORACommand(Line+1);
  }
  else if (Line[0] == 'R')
  {
    OK = ProcessRTTYCommand(Line+1);
  }
  else if (Line[0] == 'P')
  {
    OK = ProcessPredictionCommand(Line+1);
  }
  else if (Line[0] == 'F')
  {
    OK = ProcessFieldCommand(Line+1);
  }
  else if (Line[0] == 'X')
  {
    CutdownNow(5000);
  }

  if (OK)
  {
    Serial.println("*");
    #ifdef OLED
      ShowSettings();
    #endif
  }
  else
  {
    Serial.println("?");
  }
}

int ProcessFieldCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'P')
  {
    GPS.PreviousAltitude = GPS.Altitude;
    sscanf(Line+1,"%f,%f,%ld", &GPS.Latitude, &GPS.Longitude, &GPS.Altitude);
    GPS.UseHostPosition = 5;
    GPS.AscentRate = GPS.AscentRate * 0.7 + (GPS.Altitude - GPS.PreviousAltitude) * 0.3;
    OK = 1;
  }
  
  return OK;
}

void SetDefaults(void)
{
  // Common settings
  strcpy(Settings.PayloadID, "TTGO");

  const static char DefaultFieldList[] = "0123456789";   // "0123456CD";
  strcpy(Settings.FieldList, (char *)DefaultFieldList);

  // GPS Settings
  Settings.FlightModeAltitude = 2000;

  // LoRa Settings
  Settings.LoRaFrequency = 434.454;
  Settings.LoRaMode = 2;
  Settings.LoRaCycleTime = 5;
  Settings.LoRaSlot = 1;
  Settings.CallingCount = 0;
  Settings.LoRaRepeatSlot1 = -1;
  Settings.LoRaRepeatSlot2 = -1;
  Settings.UseBinaryMode = 0;
  Settings.BinaryNode = 0;

  // Cutdown Settings
  Settings.CutdownAltitude = 0;     // Disables cutdown
  Settings.CutdownPeriod = 5000;    // ms

  // Prediction Settings
  Settings.CDA = 0.7;
  Settings.PayloadWeight = 1.0;
  Settings.LandingAltitude = 100;

  // RTTY Settings
  Settings.RTTYFrequency = 434.400;
  Settings.RTTYBaudRate = 300;
  Settings.RTTYAudioShift = 488;
  Settings.RTTYCount = 0;
  Settings.RTTYEvery = 3;
  Settings.RTTYPreamble = 8;
  
  // Uplink Settings
  Settings.EnableUplink = 0;
  strcat(Settings.UplinkCode, "");
  
  SaveSettings();
}

void LoadSettings(void)
{
  unsigned int i;
  unsigned char *ptr;

  ptr = (unsigned char *)(&Settings);
  for (i=0; i<sizeof(Settings); i++, ptr++)
  {
    *ptr = EEPROM.read(i+2);
  }
}

void SaveSettings(void)
{
  unsigned int i;
  unsigned char *ptr;
  
  // Signature
  EEPROM.write(0, SIG_1);
  EEPROM.write(1, SIG_2);

  // Settings
  ptr = (unsigned char *)(&Settings);
  for (i=0; i<sizeof(Settings); i++, ptr++)
  {
    EEPROM.write(i+2, *ptr);
  }

  EEPROM.commit();
}

int ProcessCommonCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'P')
  {
    // Store payload ID
    if (strlen(Line+1) < PAYLOAD_LENGTH)
    {
      strcpy(Settings.PayloadID, Line+1);
      OK = 1;
    }
  }
  else if (Line[0] == 'F')
  {
    // Store Field List
    if (strlen(Line+1) < FIELDLIST_LENGTH)
    {
      strcpy(Settings.FieldList, Line+1);
      OK = 1;
    }
  }
  else if (Line[0] == 'R')
  {
    // Reset to default settings
    SetDefaults();
    OK = 1;
  }
  else if (Line[0] == 'S')
  {
    // Save settings to flash
    SaveSettings();
    OK = 1;
  }
  else if (Line[0] == 'E')
  {
    SendSettings();
    OK = 1;
  }
  else if (Line[0] == 'V')
  {
    Serial.printf("VER=%s\n", VERSION);
    OK = 1;
  }
  else if (Line[0] == 'C')
  {
    Serial.printf("PROD=%s\n", PRODUCT);
    OK = 1;
  }
  else if (Line[0] == 'D')
  {
    Serial.printf("DESC=%s\n", DESCRIPTION);
    OK = 1;
  }
  else if (Line[0] == 'A')
  {
    // Cutdown Altitude   
     Settings.CutdownAltitude = atol(Line+1);
     OK = 1;
  }
  else if (Line[0] == 'T')
  {
    // Cutdown Time  
     Settings.CutdownPeriod = atoi(Line+1);
     OK = 1;
  }
  else if (Line[0] == 'L')
  {
    // Enable/Disable Field List
     Settings.IncludeFieldList = Line[1] == '1';
     OK = 1;
  }

  return OK;
}

int ProcessLORACommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'F')
  {
    // New frequency
    double Frequency;
    
    Frequency = atof(Line+1);
    if ((Frequency >= 400) && (Frequency < 1000))
    {
      Settings.LoRaFrequency = Frequency;
      setupRFM98(Settings.LoRaFrequency, Settings.LoRaMode);
      OK = 1;
    }
  }
  else if (Line[0] == 'M')
  {
    // Preset Mode
    int LoRaMode = atoi(Line+1);
    
    if ((LoRaMode >= 0) && (LoRaMode <= 5))
    {
      Settings.LoRaMode = LoRaMode;
      setupRFM98(Settings.LoRaFrequency, Settings.LoRaMode);
      OK = 1;
    }
  }
  else if (Line[0] == 'C')
  {
    // Calling mode count
    int CallingCount = atoi(Line+1);
    
    if ((CallingCount >= 0) && (CallingCount <= 100))
    {
      Settings.CallingCount = CallingCount;
      OK = 1;
    }
  }
  else if (Line[0] == 'T')
  {
    int LoRaCycleTime = atoi(Line+1);
    
    if ((LoRaCycleTime >= 0) && (LoRaCycleTime <= 60))
    {
      Settings.LoRaCycleTime = LoRaCycleTime;
      OK = 1;
    }
  }
  else if (Line[0] == 'O')
  {
    int LoRaSlot = atoi(Line+1);
    
    if ((LoRaSlot >= -1) && (LoRaSlot < 60))
    {
      Settings.LoRaSlot = LoRaSlot;
      OK = 1;
    }
  }
  else if (Line[0] == 'K')
  {
    Settings.EnableUplink = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == 'U')
  {
    strncpy(Settings.UplinkCode, Line+1, sizeof(Settings.UplinkCode));
    OK = 1;
  }
  else if (Line[0] == '1')
  {
    Settings.LoRaRepeatSlot1 = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == '2')
  {
    Settings.LoRaRepeatSlot2 = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == 'Y')
  {
    Settings.UseBinaryMode = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == 'N')
  {
    Settings.BinaryNode = atoi(Line+1);
    OK = 1;
  }

  return OK;
}

int ProcessPredictionCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'C')
  {
    // CDA
    float CDA = atof(Line+1);
    
    if ((CDA > 0) && (CDA <= 10.0))
    {
      Settings.CDA = CDA;
      OK = 1;
    }
  }
  else if (Line[0] == 'W')
  {
    // Payload Weight
    float PayloadWeight = atof(Line+1);
    
    if ((PayloadWeight > 0) && (PayloadWeight <= 10.0))
    {
      Settings.PayloadWeight = PayloadWeight;
      OK = 1;
    }
  }
  else if (Line[0] == 'L')
  {
    // Landing Altitude
    int LandingAltitude = atol(Line+1);
    
    if ((LandingAltitude >= 0) && (LandingAltitude <= 20))
    {
      Settings.LandingAltitude = LandingAltitude;
      OK = 1;
    }
  }

  return OK;
}

int ProcessGPSCommand(char *Line)
{
  int OK = 0;
  
  if (Line[0] == 'F')
  {
    // Flight mode altitude
    unsigned int Altitude;
    
    Altitude = atoi(Line+1);
    
    if (Altitude < 8000)
    {
      Settings.FlightModeAltitude = Altitude;
      OK = 1;
    }
  }

  return OK;
}

int ProcessRTTYCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'F')
  {
    // New frequency
    double Frequency;
    
    Frequency = atof(Line+1);
    if ((Frequency >= 400) && (Frequency < 1000))
    {
      Settings.RTTYFrequency = Frequency;
      OK = 1;
    }
  }
  else if (Line[0] == 'B')
  {
    int BaudRate = atoi(Line+1);
    
    if ((BaudRate >= 50) && (BaudRate <= 1200))
    {
      Settings.RTTYBaudRate = BaudRate;
      OK = 1;
    }
  }
  else if (Line[0] == 'A')
  {
    int RTTYAudioShift = atoi(Line+1);
    
    if ((RTTYAudioShift >= 100) && (RTTYAudioShift <= 1200))
    {
      Settings.RTTYAudioShift = RTTYAudioShift;
      OK = 1;
    }
  }
  else if (Line[0] == 'C')
  {
    int RTTYCount = atoi(Line+1);
    
    if ((RTTYCount >= 0) && (RTTYCount <= 5))
    {
      Settings.RTTYCount = RTTYCount;
      OK = 1;
    }
  }
  else if (Line[0] == 'E')
  {
    int RTTYEvery = atoi(Line+1);
    
    if ((RTTYEvery > 0) && (RTTYEvery <= 100))
    {
      Settings.RTTYEvery = RTTYEvery;
      OK = 1;
    }
  }
  else if (Line[0] == 'P')
  {
    int RTTYPreamble = atoi(Line+1);
    
    if ((RTTYPreamble >= 4) && (RTTYPreamble <= 100))
    {
      Settings.RTTYPreamble = RTTYPreamble;
      OK = 1;
    }
  }

  return OK;
}

void SendSettings(void)
{
  Serial.printf("CP=%s\n", Settings.PayloadID);
  Serial.printf("CF=%s\n", Settings.FieldList);
  Serial.printf("CL=%d\n", Settings.IncludeFieldList ? 1 : 0);

  Serial.printf("CA=%ld\n", Settings.CutdownAltitude);
  Serial.printf("CT=%u\n", Settings.CutdownPeriod);

  Serial.printf("GF=%u\n", Settings.FlightModeAltitude);

  Serial.printf("LF=%.4f\n", Settings.LoRaFrequency);
  Serial.printf("LM=%u\n", Settings.LoRaMode);

  Serial.printf("LT=%u\n", Settings.LoRaCycleTime);
  Serial.printf("LO=%u\n", Settings.LoRaSlot);
  Serial.printf("L1=%d\n", Settings.LoRaRepeatSlot1);
  Serial.printf("L2=%d\n", Settings.LoRaRepeatSlot2);
  
  Serial.printf("LB=%d\n", Settings.UseBinaryMode);
  Serial.printf("LN=%d\n", Settings.BinaryNode);

  Serial.printf("LC=%u\n", Settings.CallingCount);

  Serial.printf("LE=%u\n", Settings.EnableUplink);
  Serial.printf("LU=%s\n", Settings.UplinkCode);

  Serial.printf("PC=%.1f\n", Settings.CDA);
  Serial.printf("PW=%.2f\n", Settings.PayloadWeight);
  Serial.printf("PL=%ld\n", Settings.LandingAltitude);
  
  Serial.printf("RF=%.4f\n", Settings.RTTYFrequency);
  Serial.printf("RB=%d\n", Settings.RTTYBaudRate);
  Serial.printf("RA=%d\n", Settings.RTTYAudioShift);
  Serial.printf("RC=%u\n", Settings.RTTYCount);
  Serial.printf("RE=%u\n", Settings.RTTYEvery);
  Serial.printf("RP=%u\n", Settings.RTTYPreamble);
}
