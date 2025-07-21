/*
A Configuration object to storing settings for Project: Traveler Flight Controllers
Copyright 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef ptConfig_h
#define ptConfig_h

#include "BoardDef.h"

#include <stdint.h>   //standard data types available, such as uint8_t
#include <arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define CONFIG_VERSION "PT0101"

class ptConfig {
  // Public Functions
  public:
      // Constructor
      ptConfig();
  
      // Public Functions
      void readEEPROM();
      void setDefaultConfig();
      void writeEEPROM();
  
      void ptConfig::readConfigParam(char *szParam, int iMaxLen);
      bool ptConfig::getConfigFromPC();
      void ptConfig::sendConfigToPC();

  
      //Getters and Setters
      char* getCallsign() { return _config.Callsign; }
      void setCallsign(char* callsign) { strcpy(_config.Callsign, callsign); }
  
      char getCallsignSSID() { return _config.CallsignSSID; }
      void setCallsignSSID(char ssid) { _config.CallsignSSID = ssid; }
  
      char* getDestination() { return _config.Destination; }
      void setDestination(char* destination) { strcpy(_config.Destination, destination); }
  
      char getDestinationSSID() { return _config.DestinationSSID; }
      void setDestinationSSID(char ssid) { _config.DestinationSSID = ssid; }
  
      char* getPath1() { return _config.Path1; }
      void setPath1(char* path1) { strcpy(_config.Path1, path1); }
  
      char getPath1SSID() { return _config.Path1SSID; }
      void setPath1SSID(char ssid) { _config.Path1SSID = ssid; }
  
      char* getPath2() { return _config.Path2; }
      void setPath2(char* path2) { strcpy(_config.Path2, path2); }
  
      char getPath2SSID() { return _config.Path2SSID; }
      void setPath2SSID(char ssid) { _config.Path2SSID = ssid; }
  
      unsigned int getDisablePathAboveAltitude() { return _config.DisablePathAboveAltitude; }
      void setDisablePathAboveAltitude(unsigned int altitude) { _config.DisablePathAboveAltitude = altitude; }
  
      char getSymbol() { return _config.Symbol; }
      void setSymbol(char symbol) { _config.Symbol = symbol; }
  
      char getSymbolPage() { return _config.SymbolPage; }
      void setSymbolPage(char page) { _config.SymbolPage = page; }
  
      byte getBeaconType() { return _config.BeaconType; }
      void setBeaconType(byte type) { _config.BeaconType = type; }
  
      unsigned long getBeaconSimpleDelay() { return _config.BeaconSimpleDelay; }
      void setBeaconSimpleDelay(unsigned long delay) { _config.BeaconSimpleDelay = delay; }
  
      unsigned int getBeaconAltitudeThreshLow() { return _config.BeaconAltitudeThreshLow; }
      void setBeaconAltitudeThreshLow(unsigned int thresh) { _config.BeaconAltitudeThreshLow = thresh; }
  
      unsigned int getBeaconAltitudeThreshHigh() { return _config.BeaconAltitudeThreshHigh; }
      void setBeaconAltitudeThreshHigh(unsigned int thresh) { _config.BeaconAltitudeThreshHigh = thresh; }
  
      unsigned long getBeaconAltitudeDelayLow() { return _config.BeaconAltitudeDelayLow; }
      void setBeaconAltitudeDelayLow(unsigned long delay) { _config.BeaconAltitudeDelayLow = delay; }
  
      unsigned long getBeaconAltitudeDelayMid() { return _config.BeaconAltitudeDelayMid; }
      void setBeaconAltitudeDelayMid(unsigned long delay) { _config.BeaconAltitudeDelayMid = delay; }
  
      unsigned long getBeaconAltitudeDelayHigh() { return _config.BeaconAltitudeDelayHigh; }
      void setBeaconAltitudeDelayHigh(unsigned long delay) { _config.BeaconAltitudeDelayHigh = delay; }
  
      unsigned int getBeaconSpeedThreshLow() { return _config.BeaconSpeedThreshLow; }
      void setBeaconSpeedThreshLow(unsigned int thresh) { _config.BeaconSpeedThreshLow = thresh; }
  
      unsigned int getBeaconSpeedThreshHigh() { return _config.BeaconSpeedThreshHigh; }
      void setBeaconSpeedThreshHigh(unsigned int thresh) { _config.BeaconSpeedThreshHigh = thresh; }
  
      unsigned long getBeaconSpeedDelayLow() { return _config.BeaconSpeedDelayLow; }
      void setBeaconSpeedDelayLow(unsigned long delay) { _config.BeaconSpeedDelayLow = delay; }
  
      unsigned long getBeaconSpeedDelayMid() { return _config.BeaconSpeedDelayMid; }
      void setBeaconSpeedDelayMid(unsigned long delay) { _config.BeaconSpeedDelayMid = delay; }
  
      unsigned long getBeaconSpeedDelayHigh() { return _config.BeaconSpeedDelayHigh; }
      void setBeaconSpeedDelayHigh(unsigned long delay) { _config.BeaconSpeedDelayHigh = delay; }
  
      byte getBeaconSlot1() { return _config.BeaconSlot1; }
      void setBeaconSlot1(byte slot) { _config.BeaconSlot1 = slot; }
  
      byte getBeaconSlot2() { return _config.BeaconSlot2; }
      void setBeaconSlot2(byte slot) { _config.BeaconSlot2 = slot; }
  
      byte getAnnounceMode() { return _config.AnnounceMode; }
      void setAnnounceMode(byte mode) { _config.AnnounceMode = mode; }
  
      char* getStatusMessage() { return _config.StatusMessage; }
      void setStatusMessage(char* message) { strcpy(_config.StatusMessage, message); }
  
      bool getStatusXmitGPSFix() { return _config.StatusXmitGPSFix; }
      void setStatusXmitGPSFix(bool xmit) { _config.StatusXmitGPSFix = xmit; }
  
      bool getStatusXmitBurstAltitude() { return _config.StatusXmitBurstAltitude; }
      void setStatusXmitBurstAltitude(bool xmit) { _config.StatusXmitBurstAltitude = xmit; }
  
      bool getStatusXmitBatteryVoltage() { return _config.StatusXmitBatteryVoltage; }
      void setStatusXmitBatteryVoltage(bool xmit) { _config.StatusXmitBatteryVoltage = xmit; }
  
      bool getStatusXmitTemp() { return _config.StatusXmitTemp; }
      void setStatusXmitTemp(bool xmit) { _config.StatusXmitTemp = xmit; }
  
      bool getStatusXmitPressure() { return _config.StatusXmitPressure; }
      void setStatusXmitPressure(bool xmit) { _config.StatusXmitPressure = xmit; }
  
      bool getStatusXmitSeconds() { return _config.StatusXmitSeconds; }
      void setStatusXmitSeconds(bool xmit) { _config.StatusXmitSeconds = xmit; }

      bool getStatusXmitCustom() { return _config.StatusXmitCustom; }
      void setStatusXmitCustom(bool xmit) { _config.StatusXmitCustom = xmit; }
  
      unsigned int getRadioTxDelay() { return _config.RadioTxDelay; }
      void setRadioTxDelay(unsigned int delay) { _config.RadioTxDelay = delay; }
  
      bool getRadioCourtesyTone() { return _config.RadioCourtesyTone; }
      void setRadioCourtesyTone(bool tone) { _config.RadioCourtesyTone = tone; }
  
      char* getRadioFreqTx() { return _config.RadioFreqTx; }
      void setRadioFreqTx(char* freq) { strcpy(_config.RadioFreqTx, freq); }
  
      char* getRadioFreqRx() { return _config.RadioFreqRx; }
      void setRadioFreqRx(char* freq) { strcpy(_config.RadioFreqRx, freq); }
  
      bool getUseGlobalFreq() { return _config.UseGlobalFreq; }
      void setUseGlobalFreq(bool value) { _config.UseGlobalFreq = value; }
      
      bool getI2cBME280() { return _config.I2cBME280; }
      void setI2cBME280(bool i2c) { _config.I2cBME280 = i2c; }

      bool getDisableGPSDuringXmit() { return _config.DisableGPSDuringXmit; }
      void setDisableGPSDuringXmit(bool delay) { _config.DisableGPSDuringXmit = delay; }

      bool getRebootHourly() { return _config.HourlyReboot; }
      void setRebootHourly(bool reboot) { _config.HourlyReboot = reboot; }      
  
      unsigned int getVoltThreshGPS() { return _config.VoltThreshGPS; }
      void setVoltThreshGPS(unsigned int thresh) { _config.VoltThreshGPS = thresh; }
  
      unsigned int getVoltThreshXmit() { return _config.VoltThreshXmit; }
      void setVoltThreshXmit(unsigned int thresh) { _config.VoltThreshXmit = thresh; }
  
      unsigned int getMinTimeBetweenXmits() { return _config.MinTimeBetweenXmits; }
      void setMinTimeBetweenXmits(unsigned int time) { _config.MinTimeBetweenXmits = time; }

      bool getDelayXmitUntilGPSFix() { return _config.DelayXmitUntilGPSFix; }
      void setDelayXmitUntilGPSFix(bool delay) { _config.DelayXmitUntilGPSFix = delay; }

      unsigned int getCheckSum() { return _config.CheckSum; }
      void setCheckSum(unsigned int sum) { _config.CheckSum = sum; }
  
  private:
    // Private Variables

    struct udtConfig {
        char Callsign[7];    //6 digit callsign + Null
        char CallsignSSID;
        char Destination[7];
        char DestinationSSID;    //Destination SSID
        char Path1[7];
        char Path1SSID;
        char Path2[7];
        char Path2SSID;
      
        unsigned int DisablePathAboveAltitude;    //the altitude to stop sending path.  If 0, then always send path defined.
      
        char Symbol;
        char SymbolPage;
      
        byte BeaconType;    //0=seconds-delay, 1=Speed Smart Beaconing, 2=Altitude Smart Beaconing, 3=Time Slots, 4=Low-power mode
        unsigned long BeaconSimpleDelay;
      
        unsigned int BeaconAltitudeThreshLow;
        unsigned int BeaconAltitudeThreshHigh;
        unsigned long BeaconAltitudeDelayLow;
        unsigned long BeaconAltitudeDelayMid;
        unsigned long BeaconAltitudeDelayHigh;
      
        unsigned int BeaconSpeedThreshLow;
        unsigned int BeaconSpeedThreshHigh;
        unsigned long BeaconSpeedDelayLow;
        unsigned long BeaconSpeedDelayMid;
        unsigned long BeaconSpeedDelayHigh;
      
        byte BeaconSlot1;
        byte BeaconSlot2;

        //Beacon Type 4 Settings
        unsigned int VoltThreshGPS;    //The voltage threshold to activate the GPS and read a position (in millivolts)
        unsigned int VoltThreshXmit;    //The voltage threshold to transmit a packet (in millivolts)
        unsigned int MinTimeBetweenXmits;    //The minimum time between transmissions (in seconds)        

        byte AnnounceMode;    //0=None, 1=LED, 2=Audio, 3=LED+Audio
      
        char StatusMessage[41];
        bool StatusXmitGPSFix;
        bool StatusXmitBurstAltitude;
        bool StatusXmitBatteryVoltage;
        bool StatusXmitTemp;
        bool StatusXmitPressure;
        bool StatusXmitSeconds;
        bool StatusXmitCustom;
        

        unsigned int RadioTxDelay;
        bool RadioCourtesyTone;
        char RadioFreqTx[9];
        char RadioFreqRx[9];
        bool UseGlobalFreq;    //Use the global frequency database based on position
      
        //Enable the BME280 temperature/pressure sensor
        bool I2cBME280;
        bool DisableGPSDuringXmit;
        bool HourlyReboot;
        bool DelayXmitUntilGPSFix;    //delay up to 1 minute for a GPS fix before transmitting

        unsigned int CheckSum;    //sum of the callsign element.  If it doesn't match, then it reinitializes the EEPROM
      } _config;
};
#endif
