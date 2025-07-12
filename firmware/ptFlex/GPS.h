/*
GPS Data Parser for Project: Traveler Flight Controllers
Copywrite 2011-2025 - Zack Clobes (W0ZC), Custom Digital Services, LLC

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef GPS_h
#define GPS_h

#include "Arduino.h"
#include <string.h>
#include <SoftwareSerial.h>

#define _MAX_SENTENCE_LEN 88
// Maximum length of the latitude string is 11 characters (ddmm.mmmmmm) plus null terminator, so 12. But APRS protocol requires 7 characters (ddmm.mm) plus null terminator, so 8.
#define _MAX_LATITUDE_LEN 12
#define _MAX_LATITUDE_XMIT_LEN 8

// Maximum length of the longitude string is 12 characters (dddmm.mmmmmm) plus null terminator, so 13. But APRS protocol requires 8 characters (dddmm.mm) plus null terminator, so 9.
#define _MAX_LONGITUDE_LEN 13
#define _MAX_LONGITUDE_XMIT_LEN 9

#define _METERS_TO_FEET 3.2808399
#define GPS_MAX_COLLECTION_TIME 3000    //number of millis to wait while collecting the two GPS strings.

#define UNKNOWN_PIN 0xFF

struct udtTime {
  int hh;
  int mm;
  int ss;
};

class GPS
{
  public:
    GPS(uint8_t pinGPSRx, uint8_t pinGPSTx, uint8_t pinGPSEnable);
    void initGPS();
    void collectGPSStrings();
    void disableGPS();
    void enableGPS(bool initGPS);  
    
    void clearInputBuffer();
    void addChar(char c);
    void getLatitude(char *sz);
    void getLongitude(char *sz);

   
    inline void getGPSTime(int *Hour, int *Minute, int *Second)
    {
    	if (_bGGAComplete || _bRMCComplete) {
    		*Hour = _currTime.hh;
    		*Minute = _currTime.mm;
    		*Second = _currTime.ss;    		
    	} else {
    		*Hour = 0;
    		*Minute = 0;
    		*Second = 0;
    	}
    }	
    
    inline int getGPSSeconds() {
      if (_bGGAComplete || _bRMCComplete) {
        return _currTime.ss;
      } else {
        return -100;      //return an invalid seconds value to show that there was a problem
      }
    }		

    inline void getGPSDate(char *sz) {
    	if (_bRMCComplete) strcpy(sz, _szGPSDate);
    	else sz[0] = '\0';
    }


    inline char LatitudeHemi() {
    	if (_bGGAComplete || _bRMCComplete) return _cLatitudeHemi;
    	else return ' ';
    }
    inline char LongitudeHemi() {
    	if (_bGGAComplete || _bRMCComplete) return _cLongitudeHemi;
    	else return ' ';
    }


    inline float Altitude() { 
    	if (_bGGAComplete) return _fAltitude;
    	else return 0;
    }
    inline float AltitudeInFeet() {
      if (_bGGAComplete) return (_fAltitude * _METERS_TO_FEET);
      else return 0;
    }
    inline float Knots() { 
    	if (_bRMCComplete) return _fKnots;
    	else return 0;
    } 
    inline float Course() { 
    	if (_bRMCComplete) return _fCourse;
    	else return 0;
    } 
        
    inline int FixQuality() { 
    	if (_bGGAComplete) return _iFixQuality;
    	else return 0;
    }
    inline int NumSats() { 
    	if (_bGGAComplete) return _iNumSats;
    	else return 0;
    }
    inline bool FixValid() { 
    	if (_bRMCComplete) return _bFixValid;
    	else return false;
    }    
    
    inline void clearSentenceFlags() {
      _bGotNewRMC = false;
      _bGotNewGGA = false;
    }
    
    inline bool gotNewRMC() { return _bGotNewRMC; }
    inline bool gotNewGGA() { return _bGotNewGGA; }
    inline unsigned long getLastDecodedMillis() { return _lastDecodedMillis; }
    inline void setDebugNEMA(bool output) { _outputNEMA = output; }

    void setDebugLevel(uint8_t level) { _debugLevel = level; }
    bool getAPRSFrequency(char *sz);

	private:
		void parseGGA();
		void parseRMC();
		//bool validateGPSSentence(char *szGPSSentence, int iNumCommas, int iMinLength);
		void getString(char *ptrHaystack, char *ptrFound, int iMaxLength);
	  char* skipToNext(char *ptr);
    uint8_t getPinMode(uint8_t pin);
	  
	  
		// Private variables
    uint8_t _pinGPSRx;
    uint8_t _pinGPSTx;
    uint8_t _pinGPSEnable;

		char _szTemp[_MAX_SENTENCE_LEN];
		char _szLatitude[_MAX_LATITUDE_LEN];
		char _cLatitudeHemi;
		char _szLongitude[_MAX_LONGITUDE_LEN];
		char _cLongitudeHemi;
		int _iFixQuality;
		bool _bFixValid;
		int _iNumSats;
		float _fAltitude;
		float _fKnots;
		float _fCourse;
		char _szGPSDate[7];
		unsigned long _lastDecodedMillis;
    bool _outputNEMA;
		udtTime _currTime;
		int _iTempPtr;
		bool _bRMCComplete;
		bool _bGGAComplete;
		bool _bGotNewRMC;
		bool _bGotNewGGA;          
		bool _bFoundStart;

    uint8_t _debugLevel;


	};
#endif
