/*
A Configuration object to storing settings for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

Version History:
Version 1.0.0 - March 9, 2025 - Initial Release.

*/

#include "ptConfig.h"


ptConfig::ptConfig() {
  this->readEEPROM();
}



/**
 * @brief  Read the configuration from the EEPROM.
 * @note   This function reads the configuration from the EEPROM.  It is called at startup to load the configuration.
 */
void ptConfig::readEEPROM() {
    for (unsigned int i=0; i<sizeof(this->_config); i++) {
        *((char*)&this->_config + i) = EEPROM.read(i);
    }

    //Check to see if the EEPROM appears to be valid
    unsigned int iCheckSum = 0;
    for (int i=0; i<7; i++) {
        iCheckSum += this->_config.Callsign[i];
    }

    if (iCheckSum != this->_config.CheckSum) {
        //we do NOT have a match - reset the Config variables
        this->setDefaultConfig();
    }
}


/**
 * @brief  Write the default configuration to the EEPROM.
 * @note   Used when the EEPROM is not initialized or has been corrupted. Checksum is based on the string inside of _config.Callsign.
 */
void ptConfig::setDefaultConfig() {

    wdt_reset();    //reset the watchdog timer

    strcpy(this->_config.Callsign, "N0CALL");
    this->_config.CallsignSSID = '0';
    strcpy(this->_config.Destination, "APRS  ");
    this->_config.DestinationSSID = '0';
    strcpy(this->_config.Path1, "WIDE2 ");
    this->_config.Path1SSID = '1';
    strcpy(this->_config.Path2, "      ");
    this->_config.Path2SSID = '0';
    this->_config.DisablePathAboveAltitude = 2000;
    this->_config.Symbol = 'O';    //letter O for balloons
    this->_config.SymbolPage = '/';
    this->_config.BeaconType = 3;    //Altitude-based delay
    this->_config.BeaconSimpleDelay = 30;
    this->_config.BeaconSpeedThreshLow = 20;
    this->_config.BeaconSpeedThreshHigh = 50;
    this->_config.BeaconSpeedDelayLow = 300;
    this->_config.BeaconSpeedDelayMid = 60;
    this->_config.BeaconSpeedDelayHigh = 120;
    this->_config.BeaconAltitudeThreshLow = 5000;
    this->_config.BeaconAltitudeThreshHigh = 20000;
    this->_config.BeaconAltitudeDelayLow  = 30;
    this->_config.BeaconAltitudeDelayMid  = 60;
    this->_config.BeaconAltitudeDelayHigh = 45;
    this->_config.BeaconSlot1 = 15;
    this->_config.BeaconSlot2 = 45;
    strcpy(this->_config.StatusMessage, "Project Traveler");
    this->_config.StatusXmitGPSFix = 1;
    this->_config.StatusXmitBurstAltitude = 0;
    this->_config.StatusXmitBatteryVoltage = 1;
    this->_config.StatusXmitTemp = 0;
    this->_config.StatusXmitPressure = 0;
    this->_config.StatusXmitSeconds = 1;
    this->_config.StatusXmitCustom = 0;

    this->_config.RadioType = 1;   //SA/DRA818V transmitter
    this->_config.RadioTxDelay = 25;
    this->_config.RadioCourtesyTone = 0;
    strcpy(this->_config.RadioFreqTx, "144.3900");
    strcpy(this->_config.RadioFreqRx, "144.3900");

    this->_config.GPSSerialBaud = 5;    //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200
    this->_config.GPSSerialInvert = 0;    //Invert the incoming signal
    this->_config.GPSType = 2;      //0=Generic NMEA, 1=UBlox, 2=ATGM332D
    this->_config.AnnounceMode = 2;    //0=No Annunciations, 1=LED, 2=LED and Piezo

    this->_config.I2cBME280 = 1;    //initialize the BME280
    this->_config.UseGlobalFreq = 0;    //use the global frequency database based on position
    this->_config.DisableGPSDuringXmit = 0;    //disable the GPS during transmission (to save power)
    this->_config.HourlyReboot = 0;    //reboot the system every hour

    this->_config.VoltThreshGPS = 3500;    //3.5V
    this->_config.VoltThreshXmit = 4100;    //4.1V
    this->_config.MinTimeBetweenXmits = 55;    //55 seconds
    this->_config.DelayXmitUntilGPSFix = 1;    //delay transmit up to 1 minute if no GPS fix

    this->_config.CheckSum = 410;		//Checksum for N0CALL

    this->writeEEPROM();
}


/**
 * @brief  Write the configuration to the EEPROM.
 * @note   This function writes the configuration to the EEPROM.  It is called after the configuration has been updated.
 */
void ptConfig::writeEEPROM() {
    for (unsigned int i=0; i<sizeof(this->_config); i++) {
        EEPROM.write(i, *((char*)&this->_config + i));
    }
}


/**
 * @brief Reads in serial line data from the PC until it finds a tab (0x09) or an End of Transmission (0x04) character.
 * @param szParam - The array to store the incoming data while it's being collected. This parameter is by reference and will be modified.
 * @param iMaxLen - The maximum length of the incoming data. Any data exceeding the iMaxLen will be discarded.
 * @note  The function will timeout if it doesn't receive anything within 1 second.
 */
void ptConfig::readConfigParam(char *szParam, int iMaxLen) {
    byte c;
    int iSize;
    unsigned long iMilliTimeout = millis() + 1000;    //wait up to 1 second for this data
  
    for (iSize=0; iSize<iMaxLen; iSize++) szParam[iSize] = 0x00;    //load the array with nulls just in case we don't find anything
    iSize = 0;    //reset to start counting up for real
  
    while (millis() < iMilliTimeout) {
  
      wdt_reset();    //reset the watchdog timer
      if (Serial.available()) {
        c = Serial.read();
  
        if (c == 0x09 || c == 0x04) {
          //this is the end of a data set
  
          return;
        }
        if (iSize < iMaxLen) {
          //only add to the return array IF there's room.  Even if there's not room, continue to parse the incoming data until a tab is found.
          szParam[iSize] = c;
          iSize++;
        }
      }
    }
    Serial.println(F("Timeout"));
  }
  

  /**
   * @brief Reads in the configuration data from the PC and loads it into the Config UDT.
   * @return True if the configuration data was successfully read in.  False if there was an error.
   * @note  
   */
  bool ptConfig::getConfigFromPC() {
  
    char szParam[45];
    unsigned long iMilliTimeout = millis() + 10000;    //wait up to 10 seconds for this data
  
    while (millis() < iMilliTimeout) {
      wdt_reset();    //reset the watchdog timer
      while (!Serial.available()) {
        //wait
        
      }
      if (Serial.available()) {
        //we have something to read
      
        if (Serial.read() == 0x01) {
    
          //we have the start to a config string
    
          this->readConfigParam(szParam, sizeof(szParam));    //should be PT01xx for the ptSolar
          if (strcmp(szParam, CONFIG_VERSION) != 0) {
            //not a config string
            Serial.println(F("No Config Type"));
            Serial.print(F(" : "));
            Serial.println(szParam);
            return false;
          }
    
          this->readConfigParam(szParam, sizeof(this->_config.Callsign));    //Callsign
          strcpy(this->_config.Callsign, szParam);
          this->readConfigParam(szParam, 1);    //Callsign SSID
          this->_config.CallsignSSID = szParam[0];
    
          this->readConfigParam(szParam, sizeof(this->_config.Destination));    //Destination
          strcpy(this->_config.Destination, szParam);
          this->readConfigParam(szParam, 1);    //SSID
          this->_config.DestinationSSID = szParam[0];
    
          this->readConfigParam(szParam, sizeof(this->_config.Path1));    //Path1
          strcpy(this->_config.Path1, szParam);
          this->readConfigParam(szParam, 1);    //SSID
          this->_config.Path1SSID = szParam[0];
    
          this->readConfigParam(szParam, sizeof(this->_config.Path2));    //Path2
          strcpy(this->_config.Path2, szParam);
          this->readConfigParam(szParam, 1);    //SSID
          this->_config.Path2SSID = szParam[0];
    
          //Cutoff altitude to stop using the path
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.DisablePathAboveAltitude = atoi(szParam);
    
          //Symbol/Page
          this->readConfigParam(szParam, 1);
          this->_config.Symbol = szParam[0];
          this->readConfigParam(szParam, 1);
          this->_config.SymbolPage = szParam[0];
    
    
          //BeaconType
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconType = atoi(szParam);
    
          //Simple Beacon Delay
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSimpleDelay = atoi(szParam);
    
          //SpeedBeacon
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSpeedThreshLow = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSpeedThreshHigh = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSpeedDelayLow = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSpeedDelayMid = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSpeedDelayHigh = atoi(szParam);
    
          //AltitudeBeacon
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconAltitudeThreshLow = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconAltitudeThreshHigh = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconAltitudeDelayLow  = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconAltitudeDelayMid  = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconAltitudeDelayHigh = atoi(szParam);
    
          //Time Slots
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSlot1 = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.BeaconSlot2 = atoi(szParam);
    
    
          //Status Message
          this->readConfigParam(szParam, sizeof(szParam));
          strcpy(this->_config.StatusMessage, szParam);
    
    
          //Misc Flags
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitGPSFix = szParam[0] == '1';
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitBurstAltitude = szParam[0] == '1';
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitBatteryVoltage = szParam[0] == '1';
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitTemp = szParam[0] == '1';
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitPressure = szParam[0] == '1';

          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitSeconds = szParam[0] == '1';

          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.StatusXmitCustom = szParam[0] == '1';
    
                
          //Radio Configuration
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.RadioType = atoi(szParam);    //0=Standard Tx-only, 1=DRA818V
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.RadioTxDelay = atoi(szParam);
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.RadioCourtesyTone = atoi(szParam);    //0=off, 1=on
    
          this->readConfigParam(szParam, sizeof(this->_config.RadioFreqTx));    //Transmit Frequency for DRA818
          strcpy(this->_config.RadioFreqTx, szParam);
          
          this->readConfigParam(szParam, sizeof(this->_config.RadioFreqRx));    //Receive Frequency for DRA818
          strcpy(this->_config.RadioFreqRx, szParam);
          
          
          //GPS Configuration
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.GPSSerialBaud = atoi(szParam);    //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.GPSSerialInvert = atoi(szParam);    //Invert the incoming signal
    
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.GPSType = atoi(szParam);        //0=Generic NMEA, 1=Ublox, 2=ATGM332D
    
    
          //Annunciator Type
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.AnnounceMode = atoi(szParam);
    
          //BME280 Configuration
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.I2cBME280 = szParam[0] == '1';

          //Global Frequency
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.UseGlobalFreq = szParam[0] == '1';

          //Disable GPS during transmission
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.DisableGPSDuringXmit = szParam[0] == '1';    //Disable the GPS during transmission

          //Hourly Reboot
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.HourlyReboot = szParam[0] == '1';    //Reboot the system every hour
    
          //Beacon Type 4 Configuration
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.VoltThreshGPS = atoi(szParam);   //Threshold for voltage before activating the GPS receiver
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.VoltThreshXmit = atoi(szParam);   //Threshold for voltage before transmitting a packet
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.MinTimeBetweenXmits = atoi(szParam);   //Minimum time between transmissions in the event we have solid voltage
          this->readConfigParam(szParam, sizeof(szParam));
          this->_config.DelayXmitUntilGPSFix = szParam[0] == '1';   //Delay up to 1 minute for a GPS fix before transmitting

          unsigned int iCheckSum = 0;
          for (int i=0; i<7; i++) {
            iCheckSum += this->_config.Callsign[i];
          }
          this->_config.CheckSum = iCheckSum;
          return true;    //done reading in the file
        }
      }
    }
    return false;
  }
  
  
  /** 
   * @brief Sends the configuration data to the PC so that the configuration can be verified and managed.
   * @note  This function will send the configuration data to the PC in a tab-delimited format. 
   */
  void ptConfig::sendConfigToPC() {
    wdt_reset();    //reset the watchdog timer
    Serial.write(0x01);
    Serial.write(CONFIG_VERSION);
    Serial.write(0x09);

    Serial.write(this->_config.Callsign);
    Serial.write(0x09);
    Serial.write(this->_config.CallsignSSID);
    Serial.write(0x09);
    Serial.write(this->_config.Destination);
    Serial.write(0x09);
    Serial.write(this->_config.DestinationSSID);
    Serial.write(0x09);
    Serial.write(this->_config.Path1);
    Serial.write(0x09);
    Serial.write(this->_config.Path1SSID);
    Serial.write(0x09);
    Serial.write(this->_config.Path2);
    Serial.write(0x09);
    Serial.write(this->_config.Path2SSID);
    Serial.write(0x09);

    //Allow to disable the path above certain altitude
    Serial.print(this->_config.DisablePathAboveAltitude, DEC);
    Serial.write(0x09);

    //Symbol
    Serial.write(this->_config.Symbol);
    Serial.write(0x09);
    Serial.write(this->_config.SymbolPage);
    Serial.write(0x09);

    //Beacon Type
    Serial.print(this->_config.BeaconType, DEC);
    Serial.write(0x09);

    //Beacon - Simple Delay
    Serial.print(this->_config.BeaconSimpleDelay, DEC);
    Serial.write(0x09);


    //Beacon - Speed Beaconing
    Serial.print(this->_config.BeaconSpeedThreshLow, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconSpeedThreshHigh, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconSpeedDelayLow, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconSpeedDelayMid, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconSpeedDelayHigh, DEC);
    Serial.write(0x09);

    //Beacon - Altitude Beaconing
    Serial.print(this->_config.BeaconAltitudeThreshLow, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconAltitudeThreshHigh, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconAltitudeDelayLow, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconAltitudeDelayMid, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconAltitudeDelayHigh, DEC);
    Serial.write(0x09);

    //Beacon - Time Slots
    Serial.print(this->_config.BeaconSlot1, DEC);
    Serial.write(0x09);
    Serial.print(this->_config.BeaconSlot2, DEC);
    Serial.write(0x09);

    wdt_reset();    //reset the watchdog timer
    Serial.flush();     //Wait for the serial port to finish sending the data

    //Status Message
    Serial.write(this->_config.StatusMessage);
    Serial.write(0x09);

    //Misc Flags
    if (this->_config.StatusXmitGPSFix) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    if (this->_config.StatusXmitBurstAltitude) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    if (this->_config.StatusXmitBatteryVoltage) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    if (this->_config.StatusXmitTemp) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    if (this->_config.StatusXmitPressure) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    if (this->_config.StatusXmitSeconds) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    if (this->_config.StatusXmitCustom) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    //Radio Parameters
    Serial.print(this->_config.RadioType, DEC);      //0=Standard, 1=SA/DRA818V
    Serial.write(0x09);
    
    Serial.print(this->_config.RadioTxDelay, DEC);
    Serial.write(0x09);
    
    if (this->_config.RadioCourtesyTone) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);               

    Serial.write(this->_config.RadioFreqTx);
    Serial.write(0x09);

    Serial.write(this->_config.RadioFreqRx);
    Serial.write(0x09);
    
    //GPS Serial Data
    Serial.print(this->_config.GPSSerialBaud, DEC);      //1=300, 2=1200, 3=2400, 4=4800, 5=9600, 6=19200
    Serial.write(0x09);

    if (this->_config.GPSSerialInvert) Serial.write("1");   //1=Invert the incoming signal
    else Serial.write("0");
    Serial.write(0x09);

    Serial.print(this->_config.GPSType, DEC);      //0=Generic NMEA, 1=Ublox, 2=ATGM332D
    Serial.write(0x09);

    //Misc System Configuration
    Serial.print(this->_config.AnnounceMode, DEC);    //0=No annunciator, 1=LED only, 2=LED and buzzer
    Serial.write(0x09);

    //BME280 Configuration
    if (this->_config.I2cBME280) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    //Use Global Frequency Database
    if (this->_config.UseGlobalFreq) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    //Disable GPS during transmission
    if (this->_config.DisableGPSDuringXmit) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    //Hourly Reboot
    if (this->_config.HourlyReboot) Serial.write("1");
    else Serial.write("0");
    Serial.write(0x09);

    wdt_reset();    //reset the watchdog timer
    Serial.flush();     //Wait for the serial port to finish sending the data

    //Beacon Type 4 Configuration
    Serial.print(this->_config.VoltThreshGPS, DEC);
    Serial.write(0x09);

    Serial.print(this->_config.VoltThreshXmit, DEC);
    Serial.write(0x09);

    Serial.print(this->_config.MinTimeBetweenXmits, DEC);
    Serial.write(0x09);

    Serial.print(this->_config.DelayXmitUntilGPSFix, DEC);

    Serial.write(0x04);      //End of string

    wdt_reset();    //reset the watchdog timer
    Serial.flush();     //Wait for the serial port to finish sending the data

  }