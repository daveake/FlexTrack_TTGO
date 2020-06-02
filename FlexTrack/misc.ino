int BuildSentence(char *TxLine, const char *PayloadID)
{
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
    char LatitudeString[16], LongitudeString[16], CRCString[8];
	
    SentenceCounter++;
	
    dtostrf(GPS.Latitude, 7, 5, LatitudeString);
    dtostrf(GPS.Longitude, 7, 5, LongitudeString);

    snprintf(TxLine,
            SENTENCE_LENGTH-6,
            "$$%s,%d,%02d:%02d:%02d,%s,%s,%05ld" EXTRA_FIELD_FORMAT,
            PayloadID,
            SentenceCounter,
	    GPS.Hours, GPS.Minutes, GPS.Seconds,
            LatitudeString,
            LongitudeString,
            GPS.Altitude
            EXTRA_FIELD_LIST 
            );
            

    Count = strlen(TxLine);

    CRC = 0xffff;           // Seed
    xPolynomial = 0x1021;
   
     for (i = 2; i < Count; i++)
     {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
     }

    TxLine[Count++] = '*';
    TxLine[Count++] = Hex((CRC >> 12) & 15);
    TxLine[Count++] = Hex((CRC >> 8) & 15);
    TxLine[Count++] = Hex((CRC >> 4) & 15);
    TxLine[Count++] = Hex(CRC & 15);
  	TxLine[Count++] = '\n';  
	  TxLine[Count++] = '\0';
	
	return strlen(TxLine) + 1;
}

int BuildLoRaCall(unsigned char *TxLine)
{
	char Frequency[8];
	
	dtostrf(LORA_FREQUENCY, 7, 3, Frequency);

  sprintf((char *)TxLine, "^^%s,%s,%d,%d,%d,%d,%d",
    			        LORA_PAYLOAD_ID, Frequency,
        			    LORA_MODE == 1 ? 1 : 0, 
        			    LORA_MODE == 1 ? ERROR_CODING_4_5 : ERROR_CODING_4_8,
        			    LORA_MODE == 2 ? BANDWIDTH_62K5 : BANDWIDTH_20K8,
        			    LORA_MODE == 2 ? SPREADING_8 : (LORA_MODE == 1 ? SPREADING_6 : SPREADING_11),
        			    LORA_MODE == 0 ? 0x08 : 0);
			
	return strlen((char *)TxLine) + 1;
}
