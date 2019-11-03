#include <axp20x.h>

/*------------------------------------------------------------------------------------------------------*\
s|                                                                                                        |
| New tracker code that can be simply rebuilt for various hardware designs.  e.g. serial or i2c GPS,     |
| NTX2B or LoRa radio.  Using LoRa, it can configured to use TDMA in which case it will accept uplinked  |
| messages and/or repeat messages from other balloons.                                                   |
|                                                                                                        |
| Configuration is using #defines near the top of the file.  These control which other modules get       |
| used and linked in, and configure those modules (e.g. radio frequency).                                |
|                                                                                                        |
| V0.00   First stab                                                                                     |
|                                                                                                        |
\*------------------------------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------------------------------

// CONFIGURATION SECTION.

// Power settings
#define POWERSAVING	                      // Comment out to disable GPS power saving

// LORA settings
#define LORA_PAYLOAD_ID   "TTGO"            // Do not use spaces.
#define LORA_SLOT            -1
#define LORA_REPEAT_SLOT_1   -1
#define LORA_REPEAT_SLOT_2   -1

#define LORA_RTTY_FREQ    434.400               // For devices that are frequency-agile
#define LORA_RTTY_BAUD       300
#define LORA_RTTY_SHIFT      488
#define LORA_RTTY_COUNT       0           // n RTTY packets.  Set to 0 to disable
#define LORA_RTTY_EVERY       3           // After every n LoRa packets
#define LORA_RTTY_PREAMBLE    8

#define LORA_TIME_INDEX      2
#define LORA_TIME_MUTLIPLER  2
#define LORA_TIME_OFFSET     1
#define LORA_PACKET_TIME    500
#define LORA_FREQUENCY       434.450
#define LORA_OFFSET           0         // Frequency to add in kHz to make Tx frequency accurate

#define LORA_ID              0
#define LORA_CYCLETIME       0                // Set to zero to send continuously
#define LORA_MODE            2
#define LORA_BINARY          0
#define LORA_CALL_FREQ 		433.650
#define LORA_CALL_MODE		 5				
#define LORA_CALL_COUNT		 10				// Set to zero to disable calling mode


// Cutdown settings
// #define CUTDOWN             A2


//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------

// HARDWARE DEFINITION

#define LORA_NSS           18                // Comment out to disable LoRa code
#define LORA_RESET         14                // Comment out if not connected
#define LORA_DIO0          26                
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI

//------------------------------------------------------------------------------------------------------


#ifndef DEBUG_SERIAL
  #define DEBUG_SERIAL Serial
#endif

#define EXTRA_FIELD_FORMAT    ",%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define EXTRA_FIELD_LIST           ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites

// #define EXTRA_FIELD_FORMAT      ",%d,%d,%d,%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
// #define EXTRA_FIELD_LIST            ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites, DS18B20_Temperatures[0], Channel0Average, GPS.CutdownStatus
                                                                // List of variables/expressions for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define SENTENCE_LENGTH      100                  // This is more than sufficient for the standard sentence.  Extend if needed; shorten if you are tight on memory.

AXP20X_Class axp;

    /*
            "$%s,%d,%02d:%02d:%02d,%s,%s,%05.5u,%d,%d,%d",
            PAYLOAD_ID,
            SentenceCounter,
	    GPS.Hours, GPS.Minutes, GPS.Seconds,
            LatitudeString,
            LongitudeString,
            GPS.Altitude,
            (int)((GPS.Speed * 13) / 7),
            GPS.Direction,
            GPS.Satellites);
    */


//------------------------------------------------------------------------------------------------------
//
//  Globals

struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	BiSeconds;
	float		Latitude;
	float		Longitude;
	int32_t  	Altitude;
};  //  __attribute__ ((packed));

struct TGPS
{
  int Hours, Minutes, Seconds;
  unsigned long SecondsInDay;					// Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  int Speed;
  int Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
  int CutdownStatus;
} GPS;


int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------

void setup()
{
  // Serial port(s)
  
  #ifdef GPS_SERIAL
    GPS_SERIAL.begin(9600);
  #endif
  
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(9600);
    Serial.println("");
    Serial.print("FlexTrack Flight Computer, payload ID(s)");
    #ifdef LORA_NSS
      Serial.print(' ');
      Serial.print(LORA_PAYLOAD_ID);
    #endif  
      
    Serial.println("");
    Serial.println("");
  #endif

  Serial.println(F("Serial GPS"));

#ifdef LORA_NSS
  Serial.println(F("LoRa telemetry enabled"));
#endif

  // SetupLEDs();
  
#ifdef CUTDOWN
  SetupCutdown();
#endif

  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  axp.setDCDC1Voltage(3300);
  
  Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
  Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
  Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
  Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
  Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
  Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

  if (axp.isChargeing()) {
      Serial.println("Charging");
  }  
  
  SetupGPS();
  
  // SetupADC();
  
  SetupLoRa();

#ifdef WIREBUS
  // Setupds18b20();
#endif
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
}
